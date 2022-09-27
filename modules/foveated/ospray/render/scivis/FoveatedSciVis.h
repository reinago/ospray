// Copyright 2020-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

// ospray
#include "../FoveatedRenderer.h"

namespace ospray {

namespace foveated {

struct FoveatedSciVis : public FoveatedRenderer
{
  FoveatedSciVis();
  std::string toString() const override;
  void commit() override;
  void *beginFrame(FrameBuffer *, World *) override;

 private:
  bool visibleLights{false};
  bool scannedVisibleLightList{true};
};

} // namespace foveated
} // namespace ospray
