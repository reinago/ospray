// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "FoveatedRenderer.ih"
#include "camera/Camera.ih"
#include "common/World.ih"
#include "math/random.ih"
#include "pf/PixelFilter.ih"
#include "render/util.ih"
#include "fb/FrameBufferView.ih"

inline void FoveatedRenderer_clearAux(const uniform vec3f &aux,
    const varying uint32 i, vec3f *uniform buffer) {
  // Clear the accumulated value.
  buffer[i] = aux;
}

inline void FoveatedRenderer_clearColour(const uniform vec3f &rgb,
    const uniform float alpha, const varying uint32 i,
    FrameBufferView *uniform fbV) {
  // Check the colour format of the framebuffer.
  if (fbV->colorBufferFormat == OSP_FB_RGBA8) {
    // rgba8
    uint32 *uniform colour = (uint32 * uniform) fbV->colorBuffer;
    if (!colour) {
      return;
    }
    //colour[i] = cvt_uint32(make_vec4f(rgb, alpha));

  } else if (fbV->colorBufferFormat == OSP_FB_SRGBA) {
    // srgba8
    uint32 *uniform colour = (uint32 * uniform) fbV->colorBuffer;
    if (!colour) {
      return;
    }
    //colour[i] = linear_to_srgba8(make_vec4f(make_vec4f(rgb, alpha)));

  } else {
    vec4f *uniform colour = (vec4f * uniform) fbV->colorBuffer;
    if (!colour) {
      return;
    }
    colour[i] = make_vec4f(rgb, alpha);
  }
}

inline void FoveatedRenderer_clearDepth(const uniform float depth,
    const varying uint32 i, float *uniform buffer) {
  // Clear the accumulated value.
  buffer[i] = depth;
}

inline void FoveatedRenderer_writeAux(const varying vec3f &aux,
    const varying uint32 i, const uniform float accumIDF,
    const uniform float accScale, vec3f *uniform buffer) {
  // Check if there is already an accumulated value.
  varying vec3f acc = make_vec3f(0.f);
  if (accumIDF > 0.0f) {
    acc = buffer[i];
  }

  // Update the accumulated value.
  unmasked
  {
    acc.x = (acc.x * accumIDF + aux.x) * accScale;
    acc.y = (acc.y * accumIDF + aux.y) * accScale;
    acc.z = (acc.z * accumIDF + aux.z) * accScale;
  }
  buffer[i] = acc;
}

inline void FoveatedRenderer_writeColour(const varying vec3f &rgb,
    const varying float alpha, const varying uint32 i, const uniform float accumIDF,
    const uniform float accScale, FrameBufferView *uniform fbV) {
  // Check the colour format of the framebuffer.
  if (fbV->colorBufferFormat == OSP_FB_RGBA8) {
    // rgba8
    uint32 *uniform colour = (uint32 * uniform) fbV->colorBuffer;
    if (!colour) {
      return;
    }
    //colour[i] = cvt_uint32(make_vec4f(rgb, alpha));

  } else if (fbV->colorBufferFormat == OSP_FB_SRGBA) {
    // srgba8
    uint32 *uniform colour = (uint32 * uniform) fbV->colorBuffer;
    if (!colour) {
      return;
    }
    //colour[i] = linear_to_srgba8(make_vec4f(make_vec4f(rgb, alpha)));

  } else {
    vec4f *uniform colour = (vec4f * uniform) fbV->colorBuffer;
    if (!colour) {
      return;
    }

    // Check if there is already an accumulated value.
    varying vec4f acc = make_vec4f(0.f);
    if (accumIDF > 0.0f) {
      acc = colour[i];
    }
    unmasked
    {
      acc.x = (acc.x * accumIDF + rgb.x) * accScale;
      acc.y = (acc.y * accumIDF + rgb.y) * accScale;
      acc.z = (acc.z * accumIDF + rgb.z) * accScale;
      acc.w = (acc.w * accumIDF + alpha) * accScale;
    }
    colour[i] = acc;
  }
}

inline void FoveatedRenderer_writeDepth(const varying float depth,
    const varying uint32 i, const uniform int accumID, float *uniform buffer) {
  // Check if there is already an accumulated value.
  varying float acc = inf;
  if (accumID > 0) {
    acc = buffer[i];
  }

  // Update the accumulated value.
  unmasked
  {
    acc = min(acc, depth);
  }
  buffer[i] = acc;
}

inline void FoveatedRenderer_clearSample(const varying vec2i &sampleCoords,
    const varying vec2i lookAtOld, const uniform vec3f &clearColour,
    const uniform float clearAlpha, const uniform float clearDepth,
    const uniform vec3f &clearNormalAlbedo, const uniform vec2i &frameSize,
    vec3f *uniform albedoBuffer, vec3f *uniform normalBuffer,
    FrameBufferView *uniform fbV)
{
  // Convert to old sample coordinates based on the current look at point.
  varying vec2i sampleCoordsOld = make_vec2i(0, 0);
  sampleCoordsOld.x = sampleCoords.x - lookAtOld.x;
  sampleCoordsOld.y = sampleCoords.y - lookAtOld.y;

  // Check if the old sample is inside the frame.
  if ((sampleCoordsOld.x < frameSize.x) & (sampleCoordsOld.y < frameSize.y) &
      (sampleCoordsOld.x >= 0) & (sampleCoordsOld.y >= 0)) {
    // Compute the index of the sample in the framebuffer.
    int32 idxOut = sampleCoordsOld.x + sampleCoordsOld.y * frameSize.x;

    // Clear the depth buffer.
    if (fbV->depthBuffer != NULL) {
      FoveatedRenderer_clearDepth(clearDepth, idxOut, fbV->depthBuffer);
    }

    // Clear the albedo vector.
    if (albedoBuffer != NULL) {
      FoveatedRenderer_clearAux(clearNormalAlbedo, idxOut, albedoBuffer);
    }

    // Clear the normal vector.
    if (normalBuffer != NULL) {
      FoveatedRenderer_clearAux(clearNormalAlbedo, idxOut, normalBuffer);
    }

    // Clear the colour to the framebuffer.
    FoveatedRenderer_clearColour(clearColour, clearAlpha, idxOut, fbV);
  }
}

unmasked void FoveatedRenderer_default_clearSamples(
    FoveatedRenderer *uniform self,
    FrameBuffer *uniform fb,
    vec3f *uniform albedoBuffer,
    vec3f *uniform normalBuffer,
    uniform vec2i *uniform lookAtOld,
    uniform int lookAtCnt,
    void *uniform perFrameData,
    uniform int taskIndex)
{
  const vec2i *uniform samples = self->samplingData;
  const uniform int32 samplesCnt = self->samplingDataCnt;
  const uniform vec2i frameSize = fb->size;

  const uniform vec3f clearColour = self->clearColour;
  const uniform float clearAlpha = self->clearAlpha;
  const uniform float clearDepth = self->clearDepth;
  const uniform vec3f clearNormalAlbedo = self->clearNormalAlbedo;

  // Get the framebuffer view.
  FrameBufferView *uniform fbV = (FrameBufferView * uniform) fb;

  //const uniform int begin = taskIndex * RENDERTILE_PIXELS_PER_JOB;
  //const uniform int end = begin + RENDERTILE_PIXELS_PER_JOB;
  // TODO: guesswork
  const uniform int begin = taskIndex * TILE_SIZE;
  const uniform int end = begin + TILE_SIZE;

  for (uniform uint32 i = begin; i < end; i += programCount) {
    // Get the current index of the samplingData.
    const uint32 index = i + programIndex;

    // Check if the index is out of bounds.
    if (index >= samplesCnt) {
      // Skip as there is no sample for this index.
      break;
    }

    // Get the current sample position.
    vec2i sampleCoords = samples[index];

    // Iterate over the look at points.
    for (uniform int32 j = 0; j < lookAtCnt; ++j) {
      // Clear the old sample.
      FoveatedRenderer_clearSample(sampleCoords, lookAtOld[j], clearColour,
        clearAlpha, clearDepth, clearNormalAlbedo, frameSize, albedoBuffer,
        normalBuffer, fbV);
    }
  }
}

unmasked void FoveatedRenderer_default_renderSamples(
    FoveatedRenderer *uniform self,
    FrameBuffer *uniform fb,
    Camera *uniform camera,
    World *uniform world,
    uniform int accumID,
    vec3f *uniform albedoBuffer,
    vec3f *uniform normalBuffer,
    uniform vec2i *uniform lookAt,
    uniform int lookAtCnt,
    void *uniform perFrameData,
    uniform int taskIndex)
{
  const uniform int32 spp = self->super.spp;
  const vec2i *uniform samples = self->samplingData;
  const uniform int32 samplesCnt = self->samplingDataCnt;
  const uniform vec2i frameSize = fb->size;

  // Get the framebuffer view.
  FrameBufferView *uniform fbV = (FrameBufferView * uniform) fb;

  // Convert the accumID to float and compute the accum scale.
  const uniform float accumIDF = accumID;
  const uniform float accScale = rcpf(accumID + 1);

  ScreenSample screenSample;
  screenSample.z = inf;
  screenSample.alpha = 0.f;

  CameraSample cameraSample;

  //const uniform int begin = taskIndex * RENDERTILE_PIXELS_PER_JOB;
  //const uniform int end = begin + RENDERTILE_PIXELS_PER_JOB;
  // TODO: guesswork
  const uniform int begin = taskIndex * TILE_SIZE;
  const uniform int end = begin + TILE_SIZE;

  for (uniform uint32 i = begin; i < end; i += programCount) {
    // Get the current index of the samplingData.
    const uint32 index = i + programIndex;

    // Check if the index is out of bounds.
    if (index >= samplesCnt) {
      // Skip as there is no sample for this index.
      break;
    }

    // Get the current sample position.
    vec2i sampleCoords = samples[index];

    // Iterate over the look at points.
    for (uniform int32 j = 0; j < lookAtCnt; ++j) {
      // Convert to sample coordinates based on the current look at point.
      screenSample.sampleID.x = sampleCoords.x - lookAt[j].x;
      screenSample.sampleID.y = sampleCoords.y - lookAt[j].y;

      // Check if the sample is inside the frame.
      if ((screenSample.sampleID.x >= frameSize.x)
          | (screenSample.sampleID.y >= frameSize.y)) {
        continue;
      }
      if ((screenSample.sampleID.x < 0)
          | (screenSample.sampleID.y < 0)) {
        continue;
      }

      // set ray t value for early ray termination (from maximum depth texture)
      vec2f center =
          make_vec2f(screenSample.sampleID.x, screenSample.sampleID.y) + 0.5f;
      const float tMax = Renderer_getMaxDepth(&self->super, center * fb->rcpSize);

      vec3f col = make_vec3f(0.f);
      float alpha = 0.f;
      float depth = inf;
      vec3f normal = make_vec3f(0.f);
      vec3f albedo = make_vec3f(0.f);

      int startSampleID = max(accumID, 0) * spp;

      for (uniform uint32 s = 0; s < spp; s++) {
        const float pixel_du = Halton_sample2(startSampleID + s);
        const float pixel_dv = CranleyPattersonRotation(
            Halton_sample3(startSampleID + s),
            1.f / 6.f); // rotate to sample center (0.5) of pixel for sampleID=0
        const vec2f pixelSample = make_vec2f(pixel_du, pixel_dv);

        //vec2f pfSample = pixelSample;
        //if (self->super.pixelFilter)
        //{
        //  pfSample =
        //      self->super.pixelFilter->sample(self->super.pixelFilter, pixelSample) +
        //      make_vec2f(0.5f);
        //}
        const PixelFilter *uniform pf = self->super.pixelFilter;
        vec2f pfSample = pixelSample;
        if (pf) {
            pfSample = pf->sample(pf, pixelSample) + make_vec2f(0.5f);
        }

        screenSample.sampleID.z = startSampleID + s;

        cameraSample.screen.x =
            (screenSample.sampleID.x + pfSample.x) * fb->rcpSize.x;
        cameraSample.screen.y =
            (screenSample.sampleID.y + pfSample.y) * fb->rcpSize.y;
        screenSample.pos = cameraSample.screen;

        // no DoF or MB per default
        cameraSample.lens.x = 0.5f;
        cameraSample.lens.y = 0.5f;
        cameraSample.time = 0.5f;

        camera->initRay(camera, screenSample.ray, cameraSample);
        screenSample.ray.t = min(screenSample.ray.t, tMax);

        screenSample.z = inf;
        screenSample.albedo =
            make_vec3f(Renderer_getBackground(&self->super, screenSample.pos));
        screenSample.normal = make_vec3f(0.f);

        self->super.renderSample(&self->super, fb, world, perFrameData, screenSample);
        col = col + screenSample.rgb;
        alpha += screenSample.alpha;
        depth = min(depth, screenSample.z);
        normal = normal + screenSample.normal;
        albedo = albedo + screenSample.albedo;
      }

      const float rspp = rcpf(spp);
      screenSample.rgb = col * rspp;
      screenSample.alpha = alpha * rspp;
      screenSample.z = depth;
      screenSample.normal = normal * rspp;
      screenSample.albedo = albedo * rspp;

      // Compute the index of the sample in the framebuffer.
      int32 idxOut = screenSample.sampleID.x + screenSample.sampleID.y *
        frameSize.x;

      // Write the depth buffer.
      if (fbV->depthBuffer != NULL) {
        FoveatedRenderer_writeDepth(screenSample.z, idxOut, accumID,
          fbV->depthBuffer);
      }

      // Write the albedo vector.
      if (albedoBuffer != NULL) {
        FoveatedRenderer_writeAux(screenSample.albedo, idxOut, accumIDF, accScale,
          albedoBuffer);
      }

      // Write the normal vector.
      if (normalBuffer != NULL) {
        FoveatedRenderer_writeAux(screenSample.normal, idxOut, accumIDF, accScale,
          normalBuffer);
      }

      // Write the colour to the framebuffer.
      FoveatedRenderer_writeColour(screenSample.rgb, screenSample.alpha, idxOut,
          accumIDF, accScale, fbV);
      }
  }
}

export void FoveatedRenderer_set(void *uniform _self,
    const uniform int32 spp,
    const uniform int32 maxDepth,
    const uniform float minContribution,
    const uniform vec4f &bgColor,
    void *uniform backplate,
    const uniform int32 numMaterials,
    void *uniform materials,
    void *uniform _maxDepthTexture,
    void *uniform pixelFilter,
    void *uniform samplingData,
    uniform int samplingDataCnt);

//void FoveatedRenderer_Constructor(FoveatedRenderer *uniform self,
//    void *uniform cppE)
//{
//  Renderer_Constructor(&self->super, cppE);
//
//  self->clearSamples = FoveatedRenderer_default_clearSamples;
//  self->renderSamples = FoveatedRenderer_default_renderSamples;
//
//  self->clearColour = make_vec3f(0.0f);
//  self->clearAlpha = 0.0f;
//  self->clearDepth = inf;
//  self->clearNormalAlbedo = make_vec3f(0.0f);
//
//  self->samplingData = NULL;
//  self->samplingDataCnt = 0;
//}

// Exports (called from C++) //////////////////////////////////////////////////

export void FoveatedRenderer_clearSamples(void *uniform _self,
    void *uniform _fb,
    void *uniform _albedoBuffer,
    void *uniform _normalBuffer,
    uniform vec2i *uniform lookAtOld,
    uniform int lookAtCnt,
    void *uniform perFrameData,
    uniform int jobID)
{
  FoveatedRenderer *uniform self = (FoveatedRenderer * uniform) _self;
  FrameBuffer *uniform fb = (FrameBuffer * uniform) _fb;
  vec3f *uniform albedoBuffer = (vec3f * uniform) _albedoBuffer;
  vec3f *uniform normalBuffer = (vec3f * uniform) _normalBuffer;
  self->clearSamples(self, fb, albedoBuffer, normalBuffer, lookAtOld, lookAtCnt,
    perFrameData, jobID);
}

export void FoveatedRenderer_renderSamples(void *uniform _self,
    void *uniform _fb,
    void *uniform _camera,
    void *uniform _world,
    uniform int accumID,
    void *uniform _albedoBuffer,
    void *uniform _normalBuffer,
    uniform vec2i *uniform lookAt,
    uniform int lookAtCnt,
    void *uniform perFrameData,
    uniform int jobID)
{
  FoveatedRenderer *uniform self = (FoveatedRenderer * uniform) _self;
  FrameBuffer *uniform fb = (FrameBuffer * uniform) _fb;
  Camera *uniform camera = (Camera * uniform) _camera;
  World *uniform world = (World * uniform) _world;
  vec3f *uniform albedoBuffer = (vec3f * uniform) _albedoBuffer;
  vec3f *uniform normalBuffer = (vec3f * uniform) _normalBuffer;
  self->renderSamples(self, fb, camera, world, accumID, albedoBuffer, normalBuffer,
    lookAt, lookAtCnt, perFrameData, jobID);
}

export void FoveatedRenderer_set(void *uniform _self,
    const uniform int32 spp,
    const uniform int32 maxDepth,
    const uniform float minContribution,
    const uniform vec4f &bgColor,
    void *uniform backplate,
    const uniform int32 numMaterials,
    void *uniform materials,
    void *uniform _maxDepthTexture,
    void *uniform pixelFilter,
    void *uniform samplingData,
    uniform int samplingDataCnt)
{
  uniform FoveatedRenderer *uniform self =
    (uniform FoveatedRenderer * uniform) _self;

  self->super.spp = spp;
  self->super.maxDepth = maxDepth;
  self->super.minContribution = minContribution;
  self->super.bgColor = bgColor;
  self->super.backplate = (uniform Texture2D * uniform) backplate;
  self->super.numMaterials = numMaterials;
  self->super.material = (Material * *uniform) materials;
  self->super.maxDepthTexture = (Texture2D * uniform) _maxDepthTexture;
  self->super.pixelFilter = (PixelFilter * uniform) pixelFilter;

  self->samplingData = (vec2i * uniform)samplingData;
  self->samplingDataCnt = samplingDataCnt;
}
