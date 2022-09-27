// Copyright 2018 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "ArcballCamera.h"
// glfw
#include "GLFW/glfw3.h"
// ospray
#include "ospray/ospray_cpp.h"
#include "rkcommon/containers/TransactionalBuffer.h"
// std
#include <functional>

using namespace rkcommon::math;
using namespace ospray;

enum class OSPRayRendererType
{
  SCIVIS,
  PATHTRACER,
  AO,
  DEBUGGER,
  FOVEATEDSCIVIS,
  OTHER
};

class GLFWOSPRayWindow
{
 public:
  GLFWOSPRayWindow(const vec2i &windowSize, bool denoiser = false);

  ~GLFWOSPRayWindow();

  static GLFWOSPRayWindow *getActiveWindow();

  void mainLoop();

 protected:
  void addObjectToCommit(OSPObject obj);

  void updateCamera();

  void reshape(const vec2i &newWindowSize);
  void motion(const vec2f &position);
  void display();
  void startNewOSPRayFrame();
  void waitOnOSPRayFrame();
  void updateTitleBar();
  void buildUI();
  void commitOutstandingHandles();
  void refreshScene(bool resetCamera = false);
  void refreshFrameOperations();

  static GLFWOSPRayWindow *activeWindow;

  vec2i windowSize;
  vec2f previousMouse{-1.f};

  bool denoiserAvailable{false};
  bool updateFrameOpsNextFrame{false};
  bool denoiserEnabled{false};
  bool showAlbedo{false};
  bool showDepth{false};
  bool showPrimID{false};
  bool showGeomID{false};
  bool showInstID{false};
  bool renderSunSky{false};
  bool cancelFrameOnInteraction{false};
  bool showUnstructuredCells{false};

  // GLFW window instance
  GLFWwindow *glfwWindow = nullptr;

  // Arcball camera instance
  std::unique_ptr<ArcballCamera> arcballCamera;
  affine3f lastXfm{one};
  OSPStereoMode cameraStereoMode{OSP_STEREO_NONE};
  float cameraMotionBlur{0.0f};
  float cameraRollingShutter{0.0f};
  OSPShutterType cameraShutterType{OSP_SHUTTER_GLOBAL};
  // only one frame during movement is rendered with MB,
  // during accumulation the camera is static and thus no MB
  bool renderCameraMotionBlur{false};

  // OSPRay objects managed by this class
  cpp::Renderer rendererPT{"pathtracer"};
  cpp::Renderer rendererSV{"scivis"};
  cpp::Renderer rendererAO{"ao"};
  cpp::Renderer rendererDBG{"debug"};
  cpp::Renderer rendererFOVSV{"foveatedscivis"};
  cpp::Renderer *renderer{nullptr};
  cpp::Camera camera{"perspective"};
  cpp::World world;
  cpp::Light sunSky{"sunSky"};
  cpp::FrameBuffer framebuffer;
  cpp::Future currentFrame;
  cpp::Texture backplateTex{"texture2d"};

  vec3f bgColor{0.f};
  vec3f sunDirection{-0.25f, -1.0f, 0.0f};
  float turbidity{3.f};
  float horizonExtension{0.1f};

  std::string scene{"boxes_lit"};

  std::string curveVariant{"bspline"};

  //OSPRayRendererType rendererType{OSPRayRendererType::SCIVIS};
  //std::string rendererTypeStr{"scivis"};
  OSPRayRendererType rendererType{OSPRayRendererType::FOVEATEDSCIVIS};
  std::string rendererTypeStr{"foveatedscivis"};

  std::string pixelFilterTypeStr{"gaussian"};

  // List of OSPRay handles to commit before the next frame
  rkcommon::containers::TransactionalBuffer<OSPObject> objectsToCommit;

  // OpenGL framebuffer texture
  GLuint framebufferTexture = 0;

  // optional registered display callback, called before every display()
  std::function<void(GLFWOSPRayWindow *)> displayCallback;

  // toggles display of ImGui UI, if an ImGui callback is provided
  bool showUi = true;

  // optional registered ImGui callback, called during every frame to build UI
  std::function<void()> uiCallback;

  // FPS measurement of last frame
  float latestFPS{0.f};

  // foveated support data
  struct indexStruct
  {
    indexStruct(INT32 x, INT32 y)
    {
      coord = {{x, y}};
    }
    std::array<INT32, 2> coord;
  };
  std::vector<indexStruct> samplingMapData;
  std::vector<UINT32> neighborIdMap;
  std::vector<float> neighborWeightMap;

  // vec2i lookAt{0, 0};
  std::vector<vec2i> lookAt;
  bool lookAtChanged{false};

  size_t hassCnt{0};
};
