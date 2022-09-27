// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

// ospray
#include "FoveatedRenderer.h"
#include "common/Instance.h"
#include "common/Util.h"
#include "common/Data.h"
#include "fb/Framebuffer.h"
#include "common/World.h"
#include "camera/Camera.h"
#include "geometry/GeometricModel.h"
// ispc exports
#include "render/FoveatedRenderer_ispc.h"
// ospray
//#include "render/LoadBalancer.h"

namespace ospray {

namespace foveated {

static FactoryMap<FoveatedRenderer> g_renderersMap;

// FoveatedRenderer definitions
// ///////////////////////////////////////////////////////

FoveatedRenderer::FoveatedRenderer()
{
  managedObjectType = OSP_RENDERER;
  pixelFilter = nullptr;

  this->clear = false;
  this->lookAtCnt = 0;
}

FoveatedRenderer::~FoveatedRenderer() {}

//void FoveatedRenderer::clearSamples(FrameBuffer *fb,
//    vec3f *albedoBuffer,
//    vec3f *normalBuffer,
//    void *lookAtOld,
//    int32 lookAtCnt,
//    void *perFrameData,
//    size_t jobID) const
//{
//  if (this->clear) {
//    // Clear the current batch of samples.
//    ispc::FoveatedRenderer_clearSamples(getIE(),
//        fb->getIE(),
//        albedoBuffer,
//        normalBuffer,
//        (ispc::vec2i *)lookAtOld,
//        lookAtCnt,
//        perFrameData,
//        jobID);
//  }
//}

void FoveatedRenderer::commit()
{
  spp = std::max(1, getParam<int>("pixelSamples", 1));
  const int32 maxDepth = std::max(0, getParam<int>("maxPathLength", 20));
  const float minContribution = getParam<float>("minContribution", 0.001f);
  errorThreshold = getParam<float>("varianceThreshold", 0.f);

  maxDepthTexture = (Texture2D *)getParamObject("map_maxDepth");
  backplate = (Texture2D *)getParamObject("map_backplate");

  if (maxDepthTexture) {
    if (maxDepthTexture->format != OSP_TEXTURE_R32F
        || maxDepthTexture->filter != OSP_TEXTURE_FILTER_NEAREST) {
      static WarnOnce warning(
          "maxDepthTexture provided to the renderer "
          "needs to be of type OSP_TEXTURE_R32F and have "
          "the OSP_TEXTURE_FILTER_NEAREST flag");
    }
  }

  vec3f bgColor3 = getParam<vec3f>(
      "backgroundColor", vec3f(getParam<float>("backgroundColor", 0.f)));
  bgColor = getParam<vec4f>("backgroundColor", vec4f(bgColor3, 0.f));

  materialData = getParamDataT<Material *>("material");

  setupPixelFilter();

  if (materialData)
    ispcMaterialPtrs = createArrayOfSh<ispc::Material>(*materialData);
  else
    ispcMaterialPtrs.clear();

  // Get the sampling data.
  this->samplingData = getParam<void *>("samplingData", nullptr);
  this->samplingDataCnt = getParam<int>("samplingDataCnt", 0);
  if (this->samplingDataCnt < 0) {
    this->samplingDataCnt = 0;
  }

  {
    // Lock the access to the current look at point.
    std::unique_lock<std::mutex> lock(this->lookAtMutex, std::defer_lock);
    lock.lock();

    // Get the number of look at points.
    this->lookAtCnt = getParam<int>("lookAtCnt", 0);
    if (this->lookAtCnt < 0) {
      this->lookAtCnt = 0;
    }

    // Check the size of the look at point storage.
    if (static_cast<int>(this->lookAt.size()) < this->lookAtCnt) {
      this->lookAt.resize(static_cast<size_t>(this->lookAtCnt));
    }

    // Get the current look at points.
    auto* curLookAtDataPtr = getParam<void *>("lookAt", nullptr);
    ::memcpy(
        this->lookAt.data(), curLookAtDataPtr, sizeof(vec2i) * this->lookAtCnt);

    // Unlock the access.
    lock.unlock();
  }

  // The look at point was updated, clear the framebuffer before the next frame is
  // rendered.
  this->clear = true;

  if (getSh()) {
    ispc::FoveatedRenderer_set(getSh(),
        spp,
        maxDepth,
        minContribution,
        (ispc::vec4f &)bgColor,
        backplate ? backplate->getSh() : nullptr,
        ispcMaterialPtrs.size(),
        ispcMaterialPtrs.data(),
        maxDepthTexture ? maxDepthTexture->getSh() : nullptr,
        pixelFilter ? pixelFilter->getIE() : nullptr,
        this->samplingData,
        this->samplingDataCnt);
  }
}

void FoveatedRenderer::endFrame(FrameBuffer *fb, void *perFrameData)
{
  this->clear = false;
}

std::pair<std::vector<vec2i>, std::vector<vec2i>> FoveatedRenderer::getLookAt(
    int32 &lookAtCnt)
{
  // Lock the access to the current look at point.
  std::unique_lock<std::mutex> lock(this->lookAtMutex, std::defer_lock);
  lock.lock();

  // Resize the old lookt at points in case a new one was added.
  if (this->lookAtOld.size() < this->lookAt.size()) {
    this->lookAtOld.resize(this->lookAt.size(), vec2i(0, 0));
  }

  // Get the current and previous look at point.
  std::pair<std::vector<vec2i>, std::vector<vec2i>> retval =
      std::make_pair(this->lookAt, this->lookAtOld);

  // Get the number of lookAtPoints.
  lookAtCnt = this->lookAtCnt;

  // Remeber the old look at point.
  this->lookAtOld = this->lookAt;

  // Unlock the access.
  lock.unlock();

  // Return the current and previous look at point.
  return retval;
}

void FoveatedRenderer::renderSamples(FrameBuffer *fb,
    Camera *camera,
    World *world,
    int32 accumID,
    vec3f *albedoBuffer,
    vec3f *normalBuffer,
    void *lookAt,
    int32 lookAtCnt,
    void *perFrameData,
    size_t jobID) const
{
  // Render the current batch of samples.
  ispc::FoveatedRenderer_renderSamples(getSh(),
      fb->getSh(),
      camera->getSh(),
      world->getSh(),
      accumID,
      albedoBuffer,
      normalBuffer,
      (ispc::vec2i *)lookAt,
      lookAtCnt,
      perFrameData,
      jobID);
}

void FoveatedRenderer::setupPixelFilter()
{
  OSPPixelFilterTypes pixelFilterType =
      (OSPPixelFilterTypes)getParam<uint8_t>("pixelFilter",
          getParam<int32_t>(
              "pixelFilter", OSPPixelFilterTypes::OSP_PIXELFILTER_GAUSS));
  pixelFilter = nullptr;
  switch (pixelFilterType) {
  case OSPPixelFilterTypes::OSP_PIXELFILTER_BOX: {
    pixelFilter = rkcommon::make_unique<ospray::BoxPixelFilter>();
    break;
  }
  case OSPPixelFilterTypes::OSP_PIXELFILTER_BLACKMAN_HARRIS: {
    pixelFilter = rkcommon::make_unique<ospray::BlackmanHarrisLUTPixelFilter>();
    break;
  }
  case OSPPixelFilterTypes::OSP_PIXELFILTER_MITCHELL: {
    pixelFilter =
        rkcommon::make_unique<ospray::MitchellNetravaliLUTPixelFilter>();
    break;
  }
  case OSPPixelFilterTypes::OSP_PIXELFILTER_POINT: {
    pixelFilter = rkcommon::make_unique<ospray::PointPixelFilter>();
    break;
  }
  case OSPPixelFilterTypes::OSP_PIXELFILTER_GAUSS:
  default: {
    pixelFilter = rkcommon::make_unique<ospray::GaussianLUTPixelFilter>();
    break;
  }
  }
}

} // namespace foveated
} // namespace ospray
