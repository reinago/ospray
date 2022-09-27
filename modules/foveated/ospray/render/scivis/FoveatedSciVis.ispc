// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "render/FoveatedRenderer.ih"
// ao renderer
#include "render/scivis/SciVis.ih"
#include "FoveatedSciVis.ih"
#include "math/random.ih"
#include "math/sampling.ih"
#include "render/scivis/surfaces.ih"
#include "render/scivis/volumes.ih"
#include "fb/FrameBuffer.ih"
#include "common/World.ih"

void FoveatedSciVis_renderSample(Renderer *uniform _self,
    FrameBuffer *uniform fb,
    World *uniform world,
    void *uniform perFrameData,
    varying ScreenSample &sample)
{
  uniform FoveatedSciVis *uniform self = (uniform FoveatedSciVis * uniform) _self;

  LDSampler ldSamplerObj;
  varying LDSampler *uniform ldSampler = &ldSamplerObj;
  LDSampler_init(ldSampler,
      sample.sampleID.x + fb->size.x * sample.sampleID.y,
      sample.sampleID.z);

  uniform bool firstHit = true;
  bool allHitsFullyTransmissive = true;
  const float originalRayTFar = sample.ray.t;

  // Allocate memory for volume intervals
  VolumeIntervals volumeIntervals;
  allocVolumeIntervals(volumeIntervals);

  // This is our main ray
  Ray &ray = sample.ray;

  // First trace the ray across clipping scene to calculate ray intervals,
  // this step should keep ray structure unchanged
  RayIntervals rayIntervals;
  traceClippingRay(world, ray, rayIntervals);

  // Iterate over all translucent geometry till we are fully opaque
  vec3f outColor = make_vec3f(0.f);
  vec3f outTransmission = make_vec3f(1.f);
  while (true) {
    // Then trace normal geometry using calculated ray intervals,
    // if hit ray.t will be updated
    traceGeometryRayIntervals(world, ray, rayIntervals);

    // Determine volume intervals by tracing ray in the volume scene
    Ray volumeRay = ray;
    traceVolumeRay(world, volumeRay, volumeIntervals);

    // Sample volumes across volume intervals (in front of geometry hit)
    if (volumeIntervals.numVolumeIntervals > 0) {
      SciVisRenderContext rc;
      rc.renderer = &self->base;
      rc.fb = fb;
      rc.world = world;
      rc.sample = sample;
      rc.ldSampler = ldSampler;
      vec4f volumeColor = integrateVolumeIntervalsGradient(rc,
          volumeIntervals,
          rayIntervals,
          volumeRay,
          ldSampler,
          self->base.volumeSamplingRate,
          true);

      // Blend volume
      outColor = outColor + outTransmission * make_vec3f(volumeColor);
      outTransmission = outTransmission * volumeColor.w;
    }

    // Add contribution from visible lights, P is used by light
    // evaluation
    DifferentialGeometry dg;
    dg.P = ray.org;
    outColor = outColor + outTransmission * evaluateLights(world, dg, ray);

    // If any geometry has been hit
    const bool hadHit = hadHit(ray);
    if (hadHit) {
      // Prepare differential geometry structure
      postIntersect(world,
          &self->super.super,
          dg,
          ray,
          DG_NG | DG_NS | DG_NORMALIZE | DG_FACEFORWARD | DG_COLOR
              | DG_TEXCOORD);

      // Shade geometry
      SSI surfaceShading;
      surfaceShading =
          computeShading(&self->base, fb, world, dg, sample, ldSampler, ray.dir);

      // Initialize other per sample data with first hit values
      // In addition to considering the first hit, all previous, fully
      // transmissive hits will lead the normal/albedo to be replaced by the
      // current hit
      if (firstHit || allHitsFullyTransmissive) {
        sample.z = ray.t;
        sample.albedo = surfaceShading.albedo;
        sample.normal = dg.Ns;
        firstHit = false;
      }

      // A sequence of hits is considered fully transparent if it's total
      // normalized transmission is close to 1.0f and all previous hits were
      // fully transmissive
      allHitsFullyTransmissive = allHitsFullyTransmissive
          && surfaceShading.opacity < self->super.super.minContribution;

      // Blend with output final color
      outColor = outColor + outTransmission * surfaceShading.shadedColor;
      outTransmission = outTransmission * surfaceShading.transmission;

      // Early exit if remaining transmission is below min contribution
      // threshold
      if (luminance(outTransmission) < self->super.super.minContribution) {
        outTransmission = make_vec3f(0.f);
        break;
      }

      // Prepare ray for next loop iteration,
      // start from the last geometry hit all over to initial Tfar
      setRay(ray, ray.t + dg.epsilon, originalRayTFar);
    } else {
      // Blend background with output final color
      vec4f backgroundColor = Renderer_getBackground(&self->super.super, sample.pos);
      outColor = outColor + outTransmission * make_vec3f(backgroundColor);
      outTransmission = outTransmission * (1.f - backgroundColor.w);

      // Initialize other per sample data with first hit values
      // If the ray hits the background and comes from a fully transmissive
      // path, we overwrite normals/albedo by background values
      if (firstHit || allHitsFullyTransmissive) {
        sample.z = ray.t;
        sample.albedo = make_vec3f(backgroundColor);
        sample.normal = make_vec3f(0.f);
      }

      // No more intersections
      break;
    }
  }

  freeVolumeIntervals(volumeIntervals);
  sample.rgb = outColor;
  sample.alpha = 1.f - luminance(outTransmission);
}

// SciVis C++ interface /////////////////////////////////////////////////////

export void *uniform FoveatedSciVis_create(void *uniform cppE)
{
  uniform FoveatedSciVis *uniform self = uniform new uniform FoveatedSciVis;
  //FoveatedRenderer_Constructor(&self->super, cppE);
  self->super.super.renderSample = FoveatedSciVis_renderSample;
  return self;
}

export void FoveatedSciVis_set(void *uniform _self,
    uniform bool shadowsEnabled,
    uniform int aoSamples,
    uniform float aoRadius,
    uniform float volumeSamplingRate)
{
  uniform FoveatedSciVis *uniform self = (uniform FoveatedSciVis * uniform) _self;

  self->base.shadowsEnabled = shadowsEnabled;
  self->base.aoSamples = aoSamples;
  self->base.aoRadius = aoRadius;

  self->base.volumeSamplingRate = volumeSamplingRate;
}

vec3f SciVis_computeAO(const uniform FoveatedSciVis *uniform self,
    const FrameBuffer *uniform fb,
    const World *uniform world,
    ScreenSample &sample,
    varying LDSampler *uniform ldSampler,
    const varying DifferentialGeometry &dg,
    const uniform int sampleCnt,
    const uniform float aoRadius,
    const varying vec3i &sampleID)
{
  const uniform int accumID = reduce_max(sampleID.z) * sampleCnt;

  // init TEA RNG //
  RandomTEA rng_state;
  varying RandomTEA *const uniform rng = &rng_state;
  RandomTEA__Constructor(rng, sampleID.x, sampleID.y);
  const vec2f rot = RandomTEA__getFloats(rng);

  vec3f hits = make_vec3f(0.f);

  const linear3f localToWorld = frame(dg.Ns);

  for (uniform int i = 0; i < sampleCnt; i++) {
    // Use base-5 and base-7 Halton to avoid correlation with sample position
    const vec2f halton = make_vec2f(
        PermHalton_sample5(accumID + i), PermHalton_sample7(accumID + i));
    const vec2f r = CranleyPattersonRotation(halton, rot);
    const vec3f local_ao_dir = cosineSampleHemisphere(r);
    const vec3f ao_dir = localToWorld * local_ao_dir;

    if (dot(ao_dir, dg.Ns) < 0.01f) { // check below surface
      hits = hits + make_vec3f(1.f);
      continue;
    }

    Ray ao_ray;
    setRay(ao_ray, dg.P, ao_dir, dg.epsilon, aoRadius);

    hits = hits
        + (1.f
            - lightAlpha(&self->base,
                ao_ray,
                fb,
                world,
                sample,
                ldSampler,
                make_vec3f(1.f),
                dg.epsilon,
                0.1f));
  }

  // the cosTheta of cosineSampleHemispherePDF and dot(shadingNormal, ao_dir)
  // cancel
  return 1.0f - (hits / (float)sampleCnt);
}
