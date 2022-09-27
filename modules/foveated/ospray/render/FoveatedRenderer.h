// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "render/Renderer.h"

#include <deque>
#include <mutex>

namespace ospray {
namespace foveated {

struct FoveatedRenderer : public Renderer
{
  FoveatedRenderer();
  virtual ~FoveatedRenderer() override;

  //virtual void clearSamples(FrameBuffer *fb,
  //    vec3f *albedoBuffer,
  //    vec3f *normalBuffer,
  //    void *lookAtOld,
  //    int32 lookAtCnt,
  //    void *perFrameData,
  //    size_t jobID) const;

  virtual void commit() override;

  void endFrame(FrameBuffer *fb, void *perFrameData) override;

  std::pair<std::vector<vec2i>, std::vector<vec2i>> getLookAt(int32 &lookAtCnt);

  /*! \brief called exactly once (on each node) at the begining of each frame */
  virtual int getSamplingDataCnt(void);

  /*! \brief called by the load balancer to render one batch of "samples" */
  virtual void renderSamples(FrameBuffer *fb,
      Camera *camera,
      World *world,
      int32 accumID,
      vec3f *albedoBuffer,
      vec3f *normalBuffer,
      void *lookAt,
      int32 lookAtCnt,
      void *perFrameData,
      size_t jobID) const;

  // Data //
  bool clear;
  std::vector<vec2i> lookAt;
  int32 lookAtCnt;
  std::vector<vec2i> lookAtOld;
  void *samplingData;
  int32 samplingDataCnt;

 private:
  void setupPixelFilter();

  mutable std::mutex lookAtMutex;
};

// Inlined definitions ////////////////////////////////////////////////////////

inline int32 FoveatedRenderer::getSamplingDataCnt(void)
{
  return this->samplingDataCnt;
}

} // namespace foveated
} // namespace ospray
