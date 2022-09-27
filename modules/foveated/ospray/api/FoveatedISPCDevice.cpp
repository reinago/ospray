// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

// ospray
#include "FoveatedISPCDevice.h"
#include "camera/Camera.h"
#include "camera/registration.h"
#include "common/Data.h"
#include "common/Group.h"
#include "common/Instance.h"
#include "common/Library.h"
#include "common/Util.h"
#include "common/World.h"
#include "fb/ImageOp.h"
#include "fb/LocalFB.h"
#include "fb/registration.h"
#include "geometry/GeometricModel.h"
#include "geometry/registration.h"
#include "lights/Light.h"
#include "lights/registration.h"
#include "render/Material.h"
#include "render/RenderTask.h"
#include "../render/FoveatedRenderer.h"
#include "../render/registration.h"
#include "texture/Texture.h"
#include "texture/Texture2D.h"
#include "texture/registration.h"
#include "volume/VolumetricModel.h"
#include "volume/transferFunction/TransferFunction.h"
#include "volume/transferFunction/registration.h"

// stl
#include <algorithm>
#include <functional>
#include <map>
#include "rkcommon/tasking/tasking_system_init.h"
#include "rkcommon/utility/CodeTimer.h"

#include "api/FoveatedISPCDevice_ispc.h"

namespace ospray {
namespace foveated {

using SetParamFcn = void(OSPObject, const char *, const void *);

template <typename T>
static void setParamOnObject(OSPObject _obj, const char *p, const T &v)
{
  auto *obj = (ManagedObject *)_obj;
  obj->setParam(p, v);
}

#define declare_param_setter(TYPE)                                             \
  {                                                                            \
    OSPTypeFor<TYPE>::value, [](OSPObject o, const char *p, const void *v) {   \
      setParamOnObject(o, p, *(TYPE *)v);                                      \
    }                                                                          \
  }

#define declare_param_setter_object(TYPE)                                      \
  {                                                                            \
    OSPTypeFor<TYPE>::value, [](OSPObject o, const char *p, const void *v) {   \
      ManagedObject *obj = *(TYPE *)v;                                         \
      setParamOnObject(o, p, obj);                                             \
    }                                                                          \
  }

#define declare_param_setter_string(TYPE)                                      \
  {                                                                            \
    OSPTypeFor<TYPE>::value, [](OSPObject o, const char *p, const void *v) {   \
      const char *str = (const char *)v;                                       \
      setParamOnObject(o, p, std::string(str));                                \
    }                                                                          \
  }

static std::map<OSPDataType, std::function<SetParamFcn>> setParamFcns = {
    declare_param_setter(api::Device *),
    declare_param_setter(void *),
    declare_param_setter(bool),
    declare_param_setter_object(ManagedObject *),
    declare_param_setter_object(Camera *),
    declare_param_setter_object(Data *),
    declare_param_setter_object(FrameBuffer *),
    declare_param_setter_object(Future *),
    declare_param_setter_object(GeometricModel *),
    declare_param_setter_object(Group *),
    declare_param_setter_object(ImageOp *),
    declare_param_setter_object(Instance *),
    declare_param_setter_object(Light *),
    declare_param_setter_object(Material *),
    declare_param_setter_object(Renderer *),
    declare_param_setter_object(Texture *),
    declare_param_setter_object(TransferFunction *),
    declare_param_setter_object(Volume *),
    declare_param_setter_object(VolumetricModel *),
    declare_param_setter_object(World *),
    declare_param_setter_string(const char *),
    declare_param_setter(char *),
    declare_param_setter(char),
    declare_param_setter(vec2c),
    declare_param_setter(vec3c),
    declare_param_setter(vec4c),
    declare_param_setter(unsigned char),
    declare_param_setter(vec2uc),
    declare_param_setter(vec3uc),
    declare_param_setter(vec4uc),
    declare_param_setter(short),
    declare_param_setter(vec2s),
    declare_param_setter(vec3s),
    declare_param_setter(vec4s),
    declare_param_setter(unsigned short),
    declare_param_setter(vec2us),
    declare_param_setter(vec3us),
    declare_param_setter(vec4us),
    declare_param_setter(int),
    declare_param_setter(vec2i),
    declare_param_setter(vec3i),
    declare_param_setter(vec4i),
    declare_param_setter(unsigned int),
    declare_param_setter(vec2ui),
    declare_param_setter(vec3ui),
    declare_param_setter(vec4ui),
    declare_param_setter(int64_t),
    declare_param_setter(vec2l),
    declare_param_setter(vec3l),
    declare_param_setter(vec4l),
    declare_param_setter(uint64_t),
    declare_param_setter(vec2ul),
    declare_param_setter(vec3ul),
    declare_param_setter(vec4ul),
    declare_param_setter(float),
    declare_param_setter(vec2f),
    declare_param_setter(vec3f),
    declare_param_setter(vec4f),
    declare_param_setter(double),
    declare_param_setter(vec2d),
    declare_param_setter(vec3d),
    declare_param_setter(vec4d),
    declare_param_setter(box1i),
    declare_param_setter(box2i),
    declare_param_setter(box3i),
    declare_param_setter(box4i),
    declare_param_setter(box1f),
    declare_param_setter(box2f),
    declare_param_setter(box3f),
    declare_param_setter(box4f),
    declare_param_setter(linear2f),
    declare_param_setter(linear3f),
    declare_param_setter(affine2f),
    declare_param_setter(affine3f)};

#undef declare_param_setter

FoveatedISPCDevice::FoveatedISPCDevice()
    : loadBalancer(std::make_shared<FoveatedLoadBalancer>())
{}

FoveatedISPCDevice::~FoveatedISPCDevice()
{
  try {
    if (embreeDevice) {
      rtcReleaseDevice(embreeDevice);
    }
  } catch (...) {
    // silently move on, sometimes a pthread mutex lock fails in Embree
  }

  if (vklDevice) {
    vklReleaseDevice(vklDevice);
  }
}

static void embreeErrorFunc(void *, const RTCError code, const char *str)
{
  postStatusMsg() << "#osp: Embree internal error " << code << " : " << str;
  OSPError e =
      (code > RTC_ERROR_UNSUPPORTED_CPU) ? OSP_UNKNOWN_ERROR : (OSPError)code;
  handleError(e, "Embree internal error '" + std::string(str) + "'");
}

static void vklErrorFunc(void *, const VKLError code, const char *str)
{
  postStatusMsg() << "#osp: Open VKL internal error " << code << " : " << str;
  OSPError e =
      (code > VKL_UNSUPPORTED_CPU) ? OSP_UNKNOWN_ERROR : (OSPError)code;
  handleError(e, "Open VKL internal error '" + std::string(str) + "'");
}

void FoveatedISPCDevice::commit()
{
  Device::commit();

  tasking::initTaskingSystem(numThreads, true);

  if (!embreeDevice) {
    embreeDevice = rtcNewDevice(generateEmbreeDeviceCfg(*this).c_str());
    rtcSetDeviceErrorFunction(embreeDevice, embreeErrorFunc, nullptr);
    RTCError erc = rtcGetDeviceError(embreeDevice);
    if (erc != RTC_ERROR_NONE) {
      // why did the error function not get called !?
      postStatusMsg() << "#osp:init: embree internal error number " << erc;
      throw std::runtime_error("failed to initialize Embree");
    }
  }

  if (!vklDevice) {
    vklLoadModule("cpu_device");

    int cpu_width = ispc::FoveatedISPCDevice_programCount();
    switch (cpu_width) {
    case 4:
      vklDevice = vklNewDevice("cpu_4");
      break;
    case 8:
      vklDevice = vklNewDevice("cpu_8");
      break;
    case 16:
      vklDevice = vklNewDevice("cpu_16");
      break;
    default:
      vklDevice = vklNewDevice("cpu");
      break;
    }

    vklDeviceSetErrorCallback(vklDevice, vklErrorFunc, nullptr);
    vklDeviceSetLogCallback(
        vklDevice,
        [](void *, const char *message) {
          postStatusMsg(OSP_LOG_INFO) << message;
        },
        nullptr);

    vklDeviceSetInt(vklDevice, "logLevel", logLevel);
    vklDeviceSetInt(vklDevice, "numThreads", numThreads);

    vklCommitDevice(vklDevice);
  }
}

///////////////////////////////////////////////////////////////////////////
// Device Implementation //////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

int FoveatedISPCDevice::loadModule(const char *name)
{
  return loadLocalModule(name);
}

///////////////////////////////////////////////////////////////////////////
// OSPRay Data Arrays /////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

OSPData FoveatedISPCDevice::newSharedData(const void *sharedData,
    OSPDataType type,
    const vec3ul &numItems,
    const vec3l &byteStride)
{
  return (OSPData) new Data(sharedData, type, numItems, byteStride);
}

OSPData FoveatedISPCDevice::newData(OSPDataType type, const vec3ul &numItems)
{
  return (OSPData) new Data(type, numItems);
}

void FoveatedISPCDevice::copyData(
    const OSPData source, OSPData destination, const vec3ul &destinationIndex)
{
  Data *dst = (Data *)destination;
  dst->copy(*(Data *)source, destinationIndex);
}

///////////////////////////////////////////////////////////////////////////
// Renderable Objects /////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

OSPLight FoveatedISPCDevice::newLight(const char *type)
{
  return (OSPLight)Light::createInstance(type);
}

OSPCamera FoveatedISPCDevice::newCamera(const char *type)
{
  ospray::Camera *ret = Camera::createInstance(type);
  ret->setDevice(embreeDevice);
  return (OSPCamera)ret;
}

OSPGeometry FoveatedISPCDevice::newGeometry(const char *type)
{
  ospray::Geometry *ret = Geometry::createInstance(type);
  ret->setDevice(embreeDevice);
  return (OSPGeometry)ret;
}

OSPVolume FoveatedISPCDevice::newVolume(const char *type)
{
  ospray::Volume *ret = new Volume(type);
  ret->setDevice(embreeDevice, vklDevice);
  return (OSPVolume)ret;
}

OSPGeometricModel FoveatedISPCDevice::newGeometricModel(OSPGeometry _geom)
{
  auto *geom = (Geometry *)_geom;
  auto *model = new GeometricModel(geom);
  return (OSPGeometricModel)model;
}

OSPVolumetricModel FoveatedISPCDevice::newVolumetricModel(OSPVolume _volume)
{
  auto *volume = (Volume *)_volume;
  auto *model = new VolumetricModel(volume);
  return (OSPVolumetricModel)model;
}

///////////////////////////////////////////////////////////////////////////
// Model Meta-Data ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

OSPMaterial FoveatedISPCDevice::newMaterial(
    const char *renderer_type, const char *material_type)
{
  return (OSPMaterial)Material::createInstance(renderer_type, material_type);
}

OSPTransferFunction FoveatedISPCDevice::newTransferFunction(const char *type)
{
  return (OSPTransferFunction)TransferFunction::createInstance(type);
}

OSPTexture FoveatedISPCDevice::newTexture(const char *type)
{
  return (OSPTexture)Texture::createInstance(type);
}

///////////////////////////////////////////////////////////////////////////
// Instancing /////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

OSPGroup FoveatedISPCDevice::newGroup()
{
  ospray::Group *ret = new Group;
  ret->setDevice(embreeDevice);
  return (OSPGroup)ret;
}

OSPInstance FoveatedISPCDevice::newInstance(OSPGroup _group)
{
  auto *group = (Group *)_group;
  auto *instance = new Instance(group);
  return (OSPInstance)instance;
}

///////////////////////////////////////////////////////////////////////////
// Top-level Worlds ///////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

OSPWorld FoveatedISPCDevice::newWorld()
{
  ospray::World *ret = new World;
  ret->setDevice(embreeDevice);
  return (OSPWorld)ret;
}

box3f FoveatedISPCDevice::getBounds(OSPObject _obj)
{
  auto *obj = (ManagedObject *)_obj;
  return obj->getBounds();
}

///////////////////////////////////////////////////////////////////////////
// Object + Parameter Lifetime Management /////////////////////////////////
///////////////////////////////////////////////////////////////////////////

void FoveatedISPCDevice::setObjectParam(
    OSPObject object, const char *name, OSPDataType type, const void *mem)
{
  if (type == OSP_UNKNOWN)
    throw std::runtime_error("cannot set OSP_UNKNOWN parameter type");

  if (type == OSP_BYTE || type == OSP_RAW) {
    setParamOnObject(object, name, *(const byte_t *)mem);
    return;
  }

  setParamFcns[type](object, name, mem);
}

void FoveatedISPCDevice::removeObjectParam(OSPObject _object, const char *name)
{
  ManagedObject *object = (ManagedObject *)_object;
  ManagedObject *existing = object->getParam<ManagedObject *>(name, nullptr);
  if (existing)
    existing->refDec();
  object->removeParam(name);
}

void FoveatedISPCDevice::commit(OSPObject _object)
{
  ManagedObject *object = (ManagedObject *)_object;
  object->commit();
  object->checkUnused();
  object->resetAllParamQueryStatus();
}

void FoveatedISPCDevice::release(OSPObject _obj)
{
  ManagedObject *obj = (ManagedObject *)_obj;
  obj->refDec();
}

void FoveatedISPCDevice::retain(OSPObject _obj)
{
  ManagedObject *obj = (ManagedObject *)_obj;
  obj->refInc();
}

///////////////////////////////////////////////////////////////////////////
// FrameBuffer Manipulation ///////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

OSPFrameBuffer FoveatedISPCDevice::frameBufferCreate(
    const vec2i &size, const OSPFrameBufferFormat mode, const uint32 channels)
{
  FrameBuffer::ColorBufferFormat colorBufferFormat = mode;

  FrameBuffer *fb = new LocalFrameBuffer(size, colorBufferFormat, channels);
  return (OSPFrameBuffer)fb;
}

OSPImageOperation FoveatedISPCDevice::newImageOp(const char *type)
{
  return (OSPImageOperation)ImageOp::createInstance(type);
}

const void *FoveatedISPCDevice::frameBufferMap(
    OSPFrameBuffer _fb, OSPFrameBufferChannel channel)
{
  LocalFrameBuffer *fb = (LocalFrameBuffer *)_fb;
  return fb->mapBuffer(channel);
}

void FoveatedISPCDevice::frameBufferUnmap(const void *mapped, OSPFrameBuffer _fb)
{
  FrameBuffer *fb = (FrameBuffer *)_fb;
  fb->unmap(mapped);
}

float FoveatedISPCDevice::getVariance(OSPFrameBuffer _fb)
{
  FrameBuffer *fb = (FrameBuffer *)_fb;
  return fb->getVariance();
}

void FoveatedISPCDevice::resetAccumulation(OSPFrameBuffer _fb)
{
  LocalFrameBuffer *fb = (LocalFrameBuffer *)_fb;
  fb->clear();
}

///////////////////////////////////////////////////////////////////////////
// Frame Rendering ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

OSPRenderer FoveatedISPCDevice::newRenderer(const char *type)
{
  return (OSPRenderer)Renderer::createInstance(type);
}

OSPFuture FoveatedISPCDevice::renderFrame(OSPFrameBuffer _fb,
    OSPRenderer _renderer,
    OSPCamera _camera,
    OSPWorld _world)
{
  FrameBuffer *fb = (FrameBuffer *)_fb;
  Renderer *renderer = (Renderer *)_renderer;
  Camera *camera = (Camera *)_camera;
  World *world = (World *)_world;

  fb->setCompletedEvent(OSP_NONE_FINISHED);

  fb->refInc();
  renderer->refInc();
  camera->refInc();
  world->refInc();

  auto *f = new RenderTask(fb, [=]() {
    utility::CodeTimer timer;
    timer.start();
    loadBalancer->renderFrame(fb, renderer, camera, world);
    timer.stop();

    fb->refDec();
    renderer->refDec();
    camera->refDec();
    world->refDec();

    return timer.seconds();
  });

  return (OSPFuture)f;
}

int FoveatedISPCDevice::isReady(OSPFuture _task, OSPSyncEvent event)
{
  auto *task = (Future *)_task;
  return task->isFinished(event);
}

void FoveatedISPCDevice::wait(OSPFuture _task, OSPSyncEvent event)
{
  auto *task = (Future *)_task;
  task->wait(event);
}

void FoveatedISPCDevice::cancel(OSPFuture _task)
{
  auto *task = (Future *)_task;
  return task->cancel();
}

float FoveatedISPCDevice::getProgress(OSPFuture _task)
{
  auto *task = (Future *)_task;
  return task->getProgress();
}

float FoveatedISPCDevice::getTaskDuration(OSPFuture _task)
{
  auto *task = (Future *)_task;
  return task->getTaskDuration();
}

OSPPickResult FoveatedISPCDevice::pick(OSPFrameBuffer _fb,
    OSPRenderer _renderer,
    OSPCamera _camera,
    OSPWorld _world,
    const vec2f &screenPos)
{
  FrameBuffer *fb = (FrameBuffer *)_fb;
  Renderer *renderer = (Renderer *)_renderer;
  Camera *camera = (Camera *)_camera;
  World *world = (World *)_world;
  return renderer->pick(fb, camera, world, screenPos);
}

extern "C" OSPError OSPRAY_DLLEXPORT ospray_module_init_foveated(
    int16_t versionMajor, int16_t versionMinor, int16_t /*versionPatch*/)
{
  auto status = moduleVersionCheck(versionMajor, versionMinor);

  if (status == OSP_NO_ERROR) {
    // Run the ISPC module's initialization function as well to register local
    // types
    status = ospLoadModule("ispc");
  }

  if (status == OSP_NO_ERROR) {
    api::Device::registerType<FoveatedISPCDevice>("cpu_fov");

    registerAllFoveatedMaterials();
    registerAllFoveatedRenderers();
  }

  return status;
}

} // namespace foveated
} // namespace ospray
