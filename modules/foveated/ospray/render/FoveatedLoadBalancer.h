// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "camera/Camera.h"
#include "common/World.h"
#include "render/LoadBalancer.h"
#include "rkcommon/utility/ArrayView.h"
#include "FoveatedRenderer.h"

namespace ospray {
namespace foveated {

struct FoveatedLoadBalancer : public TiledLoadBalancer
{
  void renderFrame(FrameBuffer *fb,
      Renderer *renderer,
      Camera *camera,
      World *world) override;

  void renderSamples(FrameBuffer *fb,
      FoveatedRenderer *renderer,
      Camera *camera,
      World *world,
      void *perFrameData);

  /* Not implemented by Distributed load balancer currently,
   * this could potentially be useful to implement later to manage
   * the actual tile list rendering after computing the list of tiles
   * to be rendered by this rank in renderFrame
   */
  void renderTiles(FrameBuffer *fb,
      Renderer *renderer,
      Camera *camera,
      World *world,
      const utility::ArrayView<int> &tileIDs,
      void *perFrameData) override;

  std::string toString() const override;

  static size_t numJobs(const int sampleCnt, const int spp, int accumID);

  std::vector<int32> samplingDataDone;
};

} // namespace foveated
} // namespace ospray
