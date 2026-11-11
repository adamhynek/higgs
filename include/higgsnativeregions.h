#ifndef HIGGS_NATIVE_REGIONS_H
#define HIGGS_NATIVE_REGIONS_H
#include "higgshandapi.h"
#ifdef __cplusplus
extern "C" {
#endif
#define HIGGS_NATIVE_REGIONS_ABI_VERSION 1u
#define HIGGS_NATIVE_REGION_COUNT 3u
enum { HIGGS_REGION_MOUTH = 1u, HIGGS_REGION_RIGHT_SHOULDER = 2u,
       HIGGS_REGION_LEFT_SHOULDER = 4u, HIGGS_REGION_ALL = 7u };
enum { HIGGS_REGION_SOURCE_VALID = 1u };
enum { HIGGS_NATIVE_REGION_CAP_DETECTOR = 1u, HIGGS_NATIVE_REGION_CAP_COPIED_FRAME = 2u };
#pragma pack(push, 8)
typedef struct HIGGS_NativeRegionSphere {
    float worldCenter[3], worldRadius;
} HIGGS_NativeRegionSphere;
typedef struct HIGGS_NativeRegionFrame {
    uint32_t structSize, physicalHand, flags, reserved;
    uint64_t epoch, sampleId, observedAtNs, sourceGeneration, handGeneration;
    float hmdPosition[3], hmdRotation[9], hmdScale;
    float handPosition[3], palmPosition[3];
    HIGGS_NativeRegionSphere spheres[HIGGS_NATIVE_REGION_COUNT];
} HIGGS_NativeRegionFrame;
typedef struct HIGGS_NativeRegionResult {
    uint32_t structSize, insideMask;
    uint64_t epoch, sampleId;
} HIGGS_NativeRegionResult;
typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_NativeRegionDetector)(
    const HIGGS_NativeRegionFrame*, HIGGS_NativeRegionResult*, void*);
typedef struct HIGGS_NativeRegionsInterface {
    uint32_t structSize, abiVersion;
    uint64_t capabilities;
    HIGGS_HandStatus(HIGGS_HAND_CALL *RegisterDetector)(HIGGS_HandClient,
        HIGGS_NativeRegionDetector, void*, HIGGS_HandRegistration*);
    HIGGS_HandStatus(HIGGS_HAND_CALL *RemoveDetector)(HIGGS_HandClient, HIGGS_HandRegistration);
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopyFrame)(HIGGS_HandClient, uint32_t, HIGGS_NativeRegionFrame*);
} HIGGS_NativeRegionsInterface;
#pragma pack(pop)
/* Optional synchronous spatial-only callback, at most one active detector.
   Negotiate at PostPostLoad or later. No link or runtime Body Slots dependency.
   Coordinates/radii are Skyrim world units; rotations are row-major. HMD scale
   has already been applied to configured offsets, never to the world radii.
   Sphere order: mouth, right shoulder, left shoulder. Mouth probes palmPosition;
   shoulders probe handPosition. Native legacy containment is strict distance <
   radius. OK + exact epoch/sample and a valid mask handles the entire frame,
   including an all-outside result. Any other status/malformed result selects
   only native fallback. SOURCE_VALID means finite current-phase source data;
   it makes no tracking-freshness promise beyond this native update.
   Callback/context survive until successful RemoveDetector or hand-client
   Unregister. Those calls drain callbacks outside callbacks; callback reentry
   returns Pending on removal. Callbacks must not call HIGGS, execute gameplay,
   retain frame pointers, block, or infer input/action authority from proximity.
   sourceGeneration changes on actual HMD node replacement, configured local
   offsets/radii or HMD scale changes; ordinary head movement does not change it.
   epoch separately identifies world/load scope. Registration survives world/load
   changes; copied source frames do not.
   CopyFrame is last-observed evidence only, not a detector-invocation API. */
const HIGGS_NativeRegionsInterface* HIGGS_HAND_CALL HIGGS_QueryNativeRegionsInterface(uint32_t, uint32_t);
typedef const HIGGS_NativeRegionsInterface*(HIGGS_HAND_CALL *HIGGS_QueryNativeRegionsInterfaceFn)(uint32_t, uint32_t);
#ifdef __cplusplus
}
#endif
#endif
