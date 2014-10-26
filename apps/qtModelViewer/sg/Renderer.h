/********************************************************************* *\
 * INTEL CORPORATION PROPRIETARY INFORMATION                            
 * This software is supplied under the terms of a license agreement or  
 * nondisclosure agreement with Intel Corporation and may not be copied 
 * or disclosed except in accordance with the terms of that agreement.  
 * Copyright (C) 2014 Intel Corporation. All Rights Reserved.           
 ********************************************************************* */

#pragma once

#include "SceneGraph.h"

namespace ospray {
  namespace sg {
    struct Renderer : public embree::RefCount {
      Renderer();

      /*! re-start accumulation (for progressive rendering). make sure
          that this function gets called at lesat once every time that
          anything changes that might change the appearance of the
          converged image (e.g., camera position, scene, frame size,
          etc) */
      void restartAccumulation();

      void setWorld(const Ref<sg::World> &world);
      void setCamera(const Ref<sg::Camera> &camera);

      // -------------------------------------------------------
      // query functions
      // -------------------------------------------------------
      
      //! find the last camera in the scene graph
      sg::Camera *getLastDefinedCamera() const;
      
      //! create a default camera
      Ref<sg::Camera> createDefaultCamera();

      //! set a default camera
      void setDefaultCamera() { setCamera(createDefaultCamera()); }

      /*! render a frame. return 0 if successful, any non-zero number if not */
      virtual int renderFrame();

      // =======================================================
      // state variables
      // =======================================================
      Ref<sg::World>       world;
      Ref<sg::Camera>      camera;
      Ref<sg::FrameBuffer> frameBuffer;
      Ref<sg::Integrator>  integrator;
      // Ref<Frame>  frame;

      // state variables
      /*! all _unique_ nodes (i.e, even instanced nodes are listed
          only once */
      Serialization uniqueNodes;
      /*! _all_ nodes (i.e, instanced nodes are listed once for each
          time they are instanced */
      Serialization allNodes;

      //! accumulation ID
      size_t accumID;

    };
  }
}

