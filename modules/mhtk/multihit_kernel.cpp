/********************************************************************* *\
 * INTEL CORPORATION PROPRIETARY INFORMATION                            
 * This software is supplied under the terms of a license agreement or  
 * nondisclosure agreement with Intel Corporation and may not be copied 
 * or disclosed except in accordance with the terms of that agreement.  
 * Copyright (C) 2014 Intel Corporation. All Rights Reserved.           
 ********************************************************************* */



// this module
#include "multihit_kernel.h"
// embree (simd classes, data structures)
#include "common/simd/avxf.h"
#ifdef __AVX2__
# include "common/simd/avxi.h"
#else
# include "common/simd/avxi_emu.h"
#endif

#include "common/simd/avxb.h"
#include "common/scene.h"
#include "kernels/xeon/bvh4/bvh4_intersector8_hybrid.h"
#include "kernels/xeon/geometry/triangle8_intersector8_moeller.h"
#include "kernels/xeon/geometry/triangle4_intersector8_moeller.h"
#include "kernels/xeon/geometry/triangle1_intersector1_moeller.h"

#ifdef __AVX2__
typedef embree::avx2::BVH4Intersector8Hybrid<embree::Triangle4Intersector8MoellerTrumbore<false> > MyIsec;
#else
typedef embree::avx::BVH4Intersector8Hybrid<embree::Triangle4Intersector8MoellerTrumbore<false> > MyIsec;
#endif

/*! if defined, the multihit kernel will do some additional checks
  that we'll never add the same triangle twice, even if spatial
  splits are enabled. if 'RTC_SCENE_HIGH_QUALITY' is not specified
  (see ospray/common/model.cpp) this is not required, and only
  introduces overhead. */
//#define TOLERATE_SPATIAL_SPLIT_BVHS 1


namespace ospray {
  namespace mhtk {
    using namespace embree;

    /*! \brief Intersect a ray with the 4 triangles, and update hit array 
      
      \ingroup mhtk_module_c 
    */
    static __forceinline void MultiHit_intersect1(embree::Ray     &ray, 
                                                  const Triangle4 &tri, 
                                                  void            *geom,
                                                  size_t           &numHitsFound,
                                                  HitInfo         *hitArray,
                                                  size_t           hitArraySize)
    {
        PING;

      /* calculate denominator */
      STAT3(normal.trav_prims,1,1,1);
      const sse3f O = sse3f(ray.org);
      const sse3f D = sse3f(ray.dir);
      const sse3f C = sse3f(tri.v0) - O;
      const sse3f R = cross(D,C);
      const ssef den = dot(sse3f(tri.Ng),D);
      const ssef absDen = abs(den);
      const ssef sgnDen = signmsk(den);

      /* perform edge tests */
      const ssef U = dot(R,sse3f(tri.e2)) ^ sgnDen;
      const ssef V = dot(R,sse3f(tri.e1)) ^ sgnDen;

      /* perform backface culling */
#if defined(__BACKFACE_CULLING__)
      sseb valid = (den > ssef(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#else
      sseb valid = (den != ssef(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#endif
      if (likely(none(valid))) return;
      
      /* perform depth test */
      const ssef T = dot(sse3f(tri.Ng),C) ^ sgnDen;
      valid &= (T > absDen*ssef(ray.tnear)) & (T < absDen*ssef(ray.tfar));
      if (likely(none(valid))) return;

      /* ray masking test */
#if defined(__USE_RAY_MASK__)
      valid &= (tri.mask & ray.mask) != 0;
      if (unlikely(none(valid))) return;
#endif

      /* calculate hit information */
      const ssef rcpAbsDen = rcp(absDen);
      const ssef u = U * rcpAbsDen;
      const ssef v = V * rcpAbsDen;
      const ssef t = T * rcpAbsDen;
      //      size_t i = select_min(valid,t);
      for (int triID=0;triID<4;triID++) {
        if (!valid[triID]) continue;

        const int geomID = tri.geomID[triID];
        const int primID = tri.primID[triID];
        const float ti = t[triID];

#if TOLERATE_SPATIAL_SPLIT_BVHS
        // BUGFIX: SEEMS BVH4 HAS SAME PRIM/GEOM MULTIPLE TIMES. FIX TEMPORARILY HERE!
        bool alreadyFound = false;
        for (int i=0;i<numHitsFound;i++) 
          if (geomID == hitArray[i].geomID && primID == hitArray[i].primID) {
            alreadyFound = true; break; 
          }
        if (alreadyFound) continue;
#endif       
        
        const Vec3fa Ng = Vec3fa(tri.Ng.x[triID],tri.Ng.y[triID],tri.Ng.z[triID]);
        PING;
        PRINT(numHitsFound);
        if (numHitsFound == hitArraySize) {
          // array is already full, check if we have to add this one at all!
          if (ti < hitArray[hitArraySize-1].t) {
            // OK, we have to add...
            size_t insertPos = hitArraySize-1;
            while ((insertPos > 0) && (ti < hitArray[insertPos-1].t)) {
              hitArray[insertPos] = hitArray[insertPos-1];
              --insertPos;
            }
            
            if (insertPos < hitArraySize) {
              /* update hit array */
              hitArray[insertPos].u = u[triID];
              hitArray[insertPos].v = v[triID];
              hitArray[insertPos].primID = primID;
              hitArray[insertPos].geomID = geomID;
              hitArray[insertPos].Ng = Ng;
            }
          }
        } else {
          size_t insertPos = numHitsFound;
          ++numHitsFound;
          while ((insertPos > 0) && (ti < hitArray[insertPos-1].t)) {
            hitArray[insertPos] = hitArray[insertPos-1];
            --insertPos;
          }
          
          /* update hit array */
          hitArray[insertPos].u = u[triID];
          hitArray[insertPos].v = v[triID];
          hitArray[insertPos].primID = primID;
          hitArray[insertPos].geomID = geomID;
          hitArray[insertPos].Ng = Ng;
        }
      }
    }
    


    /*! \brief scalar single-ray implementation of the multi-hit kernel

      \ingroup mhtk_module_c  */
    size_t multiHitKernel(RTCScene      _scene,
                          Ray          &_ray,
                          HitInfo      *hitArray,
                          size_t        hitArraySize)
    {
      PING;
      PRINT(hitArraySize);

      typedef embree::Triangle4Intersector1MoellerTrumbore PrimitiveIntersector;
      typedef PrimitiveIntersector::Primitive Primitive;
      typedef BVH4::NodeRef NodeRef;
      typedef BVH4::Node Node;
      const int stackSize = 128;      
      
      embree::Ray &ray = (embree::Ray &)_ray;
      embree::Scene *scene = (embree::Scene*)_scene;
      embree::BVH4 *bvh = (embree::BVH4 *)scene->intersectors.ptr;
      const float org_ray_far = ray.tfar;

      size_t numHitsFound = 0;

      /*! perform per ray precalculations required by the primitive intersector */
      // const Precalculations pre(ray);

      /*! stack state */
      StackItemInt32<NodeRef> stack[stackSize];  //!< stack of nodes 
      StackItemInt32<NodeRef>* stackPtr = stack+1;        //!< current stack pointer
      StackItemInt32<NodeRef>* stackEnd = stack+stackSize;
      stack[0].ptr = bvh->root;
      stack[0].dist = neg_inf;
      
      /*! load the ray into SIMD registers */
      const sse3f norg(-ray.org.x,-ray.org.y,-ray.org.z);
      const Vec3fa ray_rdir = rcp_safe(ray.dir);
      const sse3f rdir(ray_rdir.x,ray_rdir.y,ray_rdir.z);
      const Vec3fa ray_org_rdir = ray.org*ray_rdir;
      const sse3f org_rdir(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);
      const ssef  ray_near(ray.tnear);
      ssef ray_far(ray.tfar);

      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = ray_rdir.x >= 0.0f ? 0*sizeof(ssef) : 1*sizeof(ssef);
      const size_t nearY = ray_rdir.y >= 0.0f ? 2*sizeof(ssef) : 3*sizeof(ssef);
      const size_t nearZ = ray_rdir.z >= 0.0f ? 4*sizeof(ssef) : 5*sizeof(ssef);
      
      /* pop loop */
      while (true) pop:
        {
          /*! pop next node */
          if (unlikely(stackPtr == stack)) break;
          stackPtr--;
          NodeRef cur = NodeRef(stackPtr->ptr);
        
          /*! if popped node is too far, pop next one */
          if (unlikely(*(float*)&stackPtr->dist > ray.tfar))
            continue;
        
          /* downtraversal loop */
          while (true)
            {
              /*! stop if we found a leaf */
              if (unlikely(cur.isLeaf())) break;
              STAT3(normal.trav_nodes,1,1,1);
          
              /*! single ray intersection with 4 boxes */
              const Node* node = cur.node();
              const size_t farX  = nearX ^ sizeof(ssef), farY  = nearY ^ sizeof(ssef), farZ  = nearZ ^ sizeof(ssef);
#if defined (__AVX2__)
              const ssef tNearX = msub(load4f((const char*)node+nearX), rdir.x, org_rdir.x);
              const ssef tNearY = msub(load4f((const char*)node+nearY), rdir.y, org_rdir.y);
              const ssef tNearZ = msub(load4f((const char*)node+nearZ), rdir.z, org_rdir.z);
              const ssef tFarX  = msub(load4f((const char*)node+farX ), rdir.x, org_rdir.x);
              const ssef tFarY  = msub(load4f((const char*)node+farY ), rdir.y, org_rdir.y);
              const ssef tFarZ  = msub(load4f((const char*)node+farZ ), rdir.z, org_rdir.z);
#else
              const ssef tNearX = (norg.x + load4f((const char*)node+nearX)) * rdir.x;
              const ssef tNearY = (norg.y + load4f((const char*)node+nearY)) * rdir.y;
              const ssef tNearZ = (norg.z + load4f((const char*)node+nearZ)) * rdir.z;
              const ssef tFarX  = (norg.x + load4f((const char*)node+farX )) * rdir.x;
              const ssef tFarY  = (norg.y + load4f((const char*)node+farY )) * rdir.y;
              const ssef tFarZ  = (norg.z + load4f((const char*)node+farZ )) * rdir.z;
#endif

#if defined(__SSE4_1__)
              const ssef tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,ray_near));
              const ssef tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,ray_far ));
              const sseb vmask = cast(tNear) > cast(tFar);
              size_t mask = movemask(vmask)^0xf;
#else
              const ssef tNear = max(tNearX,tNearY,tNearZ,ray_near);
              const ssef tFar  = min(tFarX ,tFarY ,tFarZ ,ray_far);
              const sseb vmask = tNear <= tFar;
              size_t mask = movemask(vmask);
#endif
          
              /*! if no child is hit, pop next node */
              if (unlikely(mask == 0))
                goto pop;
          
              /*! one child is hit, continue with that child */
              size_t r = __bscf(mask);
              if (likely(mask == 0)) {
                cur = node->child(r); cur.prefetch();
                assert(cur != BVH4::emptyNode);
                continue;
              }
          
              /*! two children are hit, push far child, and continue with closer child */
              NodeRef c0 = node->child(r); c0.prefetch(); const unsigned int d0 = ((unsigned int*)&tNear)[r];
              r = __bscf(mask);
              NodeRef c1 = node->child(r); c1.prefetch(); const unsigned int d1 = ((unsigned int*)&tNear)[r];
              assert(c0 != BVH4::emptyNode);
              assert(c1 != BVH4::emptyNode);
              if (likely(mask == 0)) {
                assert(stackPtr < stackEnd); 
                if (d0 < d1) { stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++; cur = c0; continue; }
                else         { stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++; cur = c1; continue; }
              }
          
              /*! Here starts the slow path for 3 or 4 hit children. We push
               *  all nodes onto the stack to sort them there. */
              assert(stackPtr < stackEnd); 
              stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++;
              assert(stackPtr < stackEnd); 
              stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++;
          
              /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */
              assert(stackPtr < stackEnd); 
              r = __bscf(mask);
              NodeRef c = node->child(r); c.prefetch(); unsigned int d = ((unsigned int*)&tNear)[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
              assert(c != BVH4::emptyNode);
              if (likely(mask == 0)) {
                sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
                cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
                continue;
              }
          
              /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
              assert(stackPtr < stackEnd); 
              r = __bscf(mask);
              c = node->child(r); c.prefetch(); d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
              assert(c != BVH4::emptyNode);
              sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
              cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
            }
        
          /*! this is a leaf node */
          STAT3(normal.trav_leaves,1,1,1);
          size_t num; Primitive* prim = (Primitive*) cur.leaf(num);
          for (int pi=0;pi<num;pi++)
            MultiHit_intersect1(ray,prim[pi],bvh->geometry,
                                numHitsFound,hitArray,hitArraySize);
        }
      AVX_ZERO_UPPER();

      return numHitsFound;
    }

    /*! \brief 8-wide version of ISPC MHTKHit
      \ingroup mhtk_module_c  */
    struct HitInfo8 {
      avxf t; 
      avxf u;
      avxf v;
      avxi primID;
      avxi geomID;
      avx3f Ng;
    };

    /*! \brief hand-optimized version of the 8-wide multi-hit kernel

      For 8-wide AVX/AVX2 targets, the ISPC multihit kernel will call
      back into this function

      \ingroup mhtk_module_c  */
    extern "C" 
    void handcoded_multiHitKernel8(void * _valid, 
                                   void * _retValue,
                                   RTCScene _scene, 
                                   void * _ray,
                                   void * _hitArray,
                                   int    hitArraySize)
    {
      avxb &valid = *(avxb*)_valid;
      avxi &retValue = *(avxi*)_retValue;
      HitInfo hitArray1[hitArraySize];
      HitInfo8 *hitArray8 = (HitInfo8 *)_hitArray;
      ospray::Ray  ray1;
      embree::Ray8 ray8 = *(embree::Ray8*)_ray;
      
      for (int lane=0;lane<8;lane++) {
        if (!valid[lane]) 
          continue;
        ray1.org.x = ray8.org.x[lane];
        ray1.org.y = ray8.org.y[lane];
        ray1.org.z = ray8.org.z[lane];
        ray1.dir.x = ray8.dir.x[lane];
        ray1.dir.y = ray8.dir.y[lane];
        ray1.dir.z = ray8.dir.z[lane];
        ray1.primID = -1;
        ray1.geomID = -1;
        ray1.t0     = ray8.tnear[lane];
        ray1.t      = ray8.tfar[lane];
        ray1.mask   = -1;
        size_t numHits = multiHitKernel(_scene,ray1,hitArray1,hitArraySize);
        for (int i=0;i<numHits;i++) {
          hitArray8[i].t[lane] = hitArray1[i].t;
          hitArray8[i].u[lane] = hitArray1[i].u;
          hitArray8[i].v[lane] = hitArray1[i].v;
          hitArray8[i].primID[lane] = hitArray1[i].primID;
          hitArray8[i].geomID[lane] = hitArray1[i].geomID;
          hitArray8[i].Ng.x[lane] = hitArray1[i].Ng.x;
          hitArray8[i].Ng.y[lane] = hitArray1[i].Ng.y;
          hitArray8[i].Ng.z[lane] = hitArray1[i].Ng.z;
        }
        retValue[lane] = numHits;
      }
    }
    
    /*! \brief module initialization function \ingroup module_mhtk */
    extern "C" void ospray_init_module_mhtk() 
    {
      printf("Loaded 'multi-hit traversal kernel' (mhtk) plugin ...\n");
    }

  }
}
