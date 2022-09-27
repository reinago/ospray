// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "FoveatedLoadBalancer.h"

#include <OSPConfig.h>

#include "api/Device.h"
#include "rkcommon/tasking/parallel_for.h"

#include "fb/LocalFB.h"

namespace ospray {
namespace foveated {

void FoveatedLoadBalancer::renderFrame(
    FrameBuffer *fb, Renderer *renderer, Camera *camera, World *world)
{
  fb->beginFrame();
  void *perFrameData = renderer->beginFrame(fb, world);

  // Check if the render is a foveated renderer.
  auto *fovRenderer = dynamic_cast<FoveatedRenderer *>(renderer);
  if (!fovRenderer) {
    // Not a foveated renderer, use the default tiled rendering.
    renderTiles(fb, renderer, camera, world, fb->getTileIDs(), perFrameData);
  } else {
    // Foveated renderer, render the samples.
    renderSamples(fb, fovRenderer, camera, world, perFrameData);
  }

  renderer->endFrame(fb, perFrameData);

  fb->setCompletedEvent(OSP_WORLD_RENDERED);
  fb->endFrame(renderer->errorThreshold, camera);
  fb->setCompletedEvent(OSP_FRAME_FINISHED);
}

void FoveatedLoadBalancer::renderSamples(FrameBuffer *fb,
    FoveatedRenderer *renderer,
    Camera *camera,
    World *world,
    void *perFrameData)
{
  auto t0 = std::chrono::high_resolution_clock::now();

  // Check if the size of the framebuffer has changed.
  auto pixelCnt = fb->getNumPixels().product();
  if (this->samplingDataDone.size() < pixelCnt) {
    this->samplingDataDone.resize(pixelCnt, -1);
  }

  // Get the current and previous look at points.
  int32 lookAtCnt = 0;
  auto lookAt = renderer->getLookAt(lookAtCnt);

  bool cancel = false;
  std::atomic<int> samplesDone{0};

  // Get the number of samples to render.
  const int32 sampleCnt = renderer->getSamplingDataCnt();
  const float rcpPixels = 1.0f / static_cast<float>(sampleCnt);

  // Get the current accumID for the framebuffer.
  int32 curAccumID = 0;
  LocalFrameBuffer *lfb = (LocalFrameBuffer *)fb;
  if (lfb != nullptr) {
    curAccumID = lfb->tileAccumID[0];
  }

  // Get the pointer to the albedo and normal buffer.
  vec3f *albedoBufferPtr = nullptr;
  vec3f *normalBufferPtr = nullptr;
  if (lfb != nullptr) {
    albedoBufferPtr = lfb->albedoBuffer.data();
    normalBufferPtr = lfb->normalBuffer.data();
  }

#ifdef OSPRAY_SERIAL_RENDERING
    tasking::serial_for(
        numJobs(sampleCnt, renderer->spp, accumID), [&](size_t tIdx) {
#else
    tasking::parallel_for(
        divRoundUp(sampleCnt / OSPRAY_RENDER_TASK_SIZE, 1), [&](size_t tIdx) {
#endif
        // Clear the samples of the current job.
        renderer->clearSamples(fb,
            albedoBufferPtr,
            normalBufferPtr,
            lookAt.second.data(),
            lookAtCnt,
            perFrameData,
            tIdx);

    });

#ifdef OSPRAY_SERIAL_RENDERING
    tasking::serial_for(
        numJobs(sampleCnt, renderer->spp, accumID), [&](size_t tIdx) {
#else
    tasking::parallel_for(FoveatedLoadBalancer::numJobs(
                              sampleCnt, renderer->spp, curAccumID),
        [&](size_t tIdx) {
#endif
        // Check if the frame was cancled.
        if (cancel) {
          return;
        }

        // Increase the number of samples that have been rendered.
        samplesDone += OSPRAY_RENDER_TASK_SIZE;

        // Render the samples of the current job.
        renderer->renderSamples(fb,
            camera,
            world,
            curAccumID,
            albedoBufferPtr,
            normalBufferPtr,
            lookAt.first.data(),
            lookAtCnt,
            perFrameData,
            tIdx);

        // Update the number of rendered samples.
        fb->reportProgress(samplesDone * rcpPixels);

        // Check if the frame was cancled.
        if (fb->frameCancelled()) {
          cancel = true;
        }
   });

  // Increase the accumID of the framebuffer. Since we do not update the tiles the
  // accumID of the first local framebuffer is used.
  lfb->tileAccumID[0]++;

  auto t1 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> render = t1 - t0;
  std::chrono::microseconds dRender =
      std::chrono::duration_cast<std::chrono::microseconds>(render);

  std::string hass = "render " + std::to_string(dRender.count()) + "\n";
  ::OutputDebugStringA(hass.c_str());
}

void FoveatedLoadBalancer::renderTiles(FrameBuffer *fb,
    Renderer *renderer,
    Camera *camera,
    World *world,
    const utility::ArrayView<int> &tileIDs,
    void *perFrameData)
{
  bool cancel = false;
  std::atomic<int> pixelsDone{0};

  const auto fbSize = fb->getNumPixels();
  const float rcpPixels = 1.0f / (TILE_SIZE * TILE_SIZE * tileIDs.size());

#ifdef OSPRAY_SERIAL_RENDERING
  tasking::serial_for(tileIDs.size(), [&](size_t taskIndex) {
#else
  tasking::parallel_for(tileIDs.size(), [&](size_t taskIndex) {
#endif
    if (cancel)
      return;
    const size_t numTiles_x = fb->getNumTiles().x;
    const size_t tile_y = tileIDs[taskIndex] / numTiles_x;
    const size_t tile_x = tileIDs[taskIndex] - tile_y * numTiles_x;
    const vec2i tileID(tile_x, tile_y);
    const int32 accumID = fb->accumID(tileID);

    // increment also for finished tiles
    vec2i numPixels = min(vec2i(TILE_SIZE), fbSize - tileID * TILE_SIZE);
    pixelsDone += numPixels.product();

    if (fb->tileError(tileID) <= renderer->errorThreshold)
      return;

#if TILE_SIZE > MAX_TILE_SIZE
    auto tilePtr = make_unique<Tile>(tileID, fbSize, accumID);
    auto &tile = *tilePtr;
#else
    Tile __aligned(64) tile(tileID, fbSize, accumID);
#endif

#ifdef OSPRAY_SERIAL_RENDERING
    tasking::serial_for(numJobs(renderer->spp, accumID), [&](size_t tIdx) {
#else
    tasking::parallel_for(
        TiledLoadBalancer::numJobs(renderer->spp, accumID), [&](size_t tIdx) {
#endif
      renderer->renderTile(fb, camera, world, perFrameData, tile, tIdx);
    });

    fb->setTile(tile);
    // TODO: Maybe change progress reporting to sum an atomic float instead?
    // with multidevice there will be many parallel updates to report progress,
    // each reporting their progress on some subset of pixels.
    fb->reportProgress(pixelsDone * rcpPixels);

    if (fb->frameCancelled())
      cancel = true;
  });
}

std::string FoveatedLoadBalancer::toString() const
{
  return "ospray::foveated::FoveatedLoadBalancer";
}

size_t FoveatedLoadBalancer::numJobs(
    const int sampleCnt, const int spp, int accumID)
{
  const int blocks =
      (accumID > 0 || spp > 0) ? 1 : std::min(1 << -2 * spp, sampleCnt);
  return divRoundUp(sampleCnt / OSPRAY_RENDER_TASK_SIZE, blocks);
}

} // namespace foveated
} // namespace ospray
