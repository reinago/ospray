## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

## Add the include directories.
ispc_include_directories(
  ${PROJECT_SOURCE_DIR}/ospray/include
  ${PROJECT_SOURCE_DIR}/ospray
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${RKCOMMON_INCLUDE_DIRS}
  ${EMBREE_INCLUDE_DIRS}
)

## Add the library.
add_library(ospray_module_foveated SHARED)

## Add the source files.
ispc_target_add_sources(ospray_module_foveated
  ${OSPRAY_RESOURCE}
  
  ## the registration.cpp files for the Cameras, ImageOps, Geometries,
  ## Lights, Materials, Renderers, Textures and TransferFunctions  
  #render/registration.cpp
  ##${PROJECT_SOURCE_DIR}/ospray/camera/registration.cpp
  ##${PROJECT_SOURCE_DIR}/ospray/lights/registration.cpp
  ##${PROJECT_SOURCE_DIR}/ospray/geometry/registration.cpp
  ##${PROJECT_SOURCE_DIR}/ospray/volume/transferFunction/registration.cpp
  ##${PROJECT_SOURCE_DIR}/ospray/texture/registration.cpp
  ##${PROJECT_SOURCE_DIR}/ospray/fb/registration.cpp
  
  ## The ISPCDevice for the foveated rendering.
  api/FoveatedISPCDevice.cpp  
  api/FoveatedISPCDevice.ispc  
  
  ## The LoadBalancer for the foveated rendering.
  render/FoveatedLoadBalancer.cpp
  
  ## Additional ispc files required for the foveated renderer. Use the
  ## files provided by the ospray_module_ispc so that there is no copy.
  #../../../ospray/render/util.ispc
  #../../../ospray/render/Renderer.ispc
  #../../../ospray/math/halton.ispc
  
  ## The foveated renderer, which is the parent class for the FoveatedSciVis,
  ## the FoveatedPathtracee, the FoveatedAO and the FoveatedDebug renderer.
  render/FoveatedRenderer.cpp
  render/FoveatedRenderer.ispc  
  
  ## Additional ispc files required for the foveated SciVis renderer. Use the
  ## files provided by the ospray_module_ispc so that there is no copy.
  ../../../ospray/render/scivis/lightAlpha.ispc
  ../../../ospray/render/scivis/surfaces.ispc
  ../../../ospray/render/scivis/volumes.ispc
  ../../../ospray/render/scivis/SciVis.ispc
  ../../../ospray/render/pathtracer/GeometryLight.ispc
  ../../../ospray/common/World.ispc
  ../../../ospray/math/sobol.ispc
  ../../../ospray/math/Distribution1D.ispc
  
  ## The foveated SciVis renderer.
  render/scivis/FoveatedSciVis.cpp
  render/scivis/FoveatedSciVis.ispc

  moduleInit.cpp
)

## Link the required external libraries.
target_link_libraries(ospray_module_foveated
  PUBLIC
    ospray_module_cpu)

## Add the include directories for the target library.
target_include_directories(ospray_module_foveated
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
    # For ISPC module generated ispc headers.
    ${PROJECT_BINARY_DIR}/ospray/
)

## Add compile definitions for the foveated rendering.
target_compile_definitions(ospray_module_foveated
  PUBLIC
    # was: RENDER_SAMPLES_PER_JOB={OSPRAY_PIXELS_PER_JOB}
    # changelog: Replace CMake variable OSPRAY_PIXELS_PER_JOB by OSPRAY_RENDER_TASK_SIZE
)

## Install the library.
ospray_install_library(ospray_module_foveated lib)