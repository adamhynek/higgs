#ifndef HIGGS_MOTION_H
#define HIGGS_MOTION_H

/* Additive read-only C ABI; independent of official001 and scoped restrictions. */
#include <stdint.h>
#if defined(_WIN32)
#define HIGGS_MOTION_CALL __cdecl
#else
#define HIGGS_MOTION_CALL
#endif
#ifdef __cplusplus
extern "C" {
#endif

#define HIGGS_MOTION_ABI_VERSION 1u
#define HIGGS_MOTION_CAP_VELOCITY UINT64_C(1)
#define HIGGS_MOTION_CAP_PRODUCER_AGE UINT64_C(2)
#define HIGGS_MOTION_CAP_PAIRED_CONTINUITY UINT64_C(4)
#define HIGGS_MOTION_CAP_FIVE_SAMPLE_HISTORY UINT64_C(8)
#define HIGGS_MOTION_HISTORY_SIZE 5u

typedef uint32_t HIGGS_MotionResult;
enum {
    HIGGS_MOTION_OK = 0, HIGGS_MOTION_INVALID_ARGUMENT = 1,
    HIGGS_MOTION_UNSUPPORTED_VERSION = 2, HIGGS_MOTION_BUFFER_TOO_SMALL = 3,
    HIGGS_MOTION_INTERNAL_ERROR = 4
};
enum { HIGGS_MOTION_LEFT = 1u, HIGGS_MOTION_RIGHT = 2u, HIGGS_MOTION_BOTH = 3u };
enum {
    HIGGS_MOTION_UNAVAILABLE = 0, HIGGS_MOTION_VALID = 1,
    HIGGS_MOTION_INVALID_POSE = 2, HIGGS_MOTION_DISCONNECTED = 3,
    HIGGS_MOTION_INVALID_CONVERSION = 4, HIGGS_MOTION_NONFINITE = 5,
    HIGGS_MOTION_WORLD_UNAVAILABLE = 6
};
enum {
    HIGGS_MOTION_WORLD_OBSERVED = 1u, HIGGS_MOTION_LOADING = 2u,
    HIGGS_MOTION_EXHAUSTED = 4u
};
enum {
    HIGGS_MOTION_LINEAR_VALID = 1u, HIGGS_MOTION_ANGULAR_VALID = 2u,
    HIGGS_MOTION_MEAN_LINEAR_VALID = 4u, HIGGS_MOTION_MEAN_SPEED_VALID = 8u,
    HIGGS_MOTION_PEAK_VALID = 16u, HIGGS_MOTION_AGE_KNOWN = 32u
};
enum { HIGGS_MOTION_SOURCE_POSE_CALLBACK = 1u, HIGGS_MOTION_NO_HARDWARE_TIMESTAMP = 2u };

#pragma pack(push, 8)
typedef struct HIGGS_MotionVector3 { float x, y, z; } HIGGS_MotionVector3;
typedef struct HIGGS_MotionHand {
    uint32_t physicalHand; /* hands[0]: HIGGS_MOTION_LEFT(1), hands[1]: HIGGS_MOTION_RIGHT(2); availableHands mask bits. */
    uint32_t status;
    uint32_t validFields;
    uint32_t sourceFlags;
    uint64_t acceptedObservation; /* Private HIGGS ordering ticket, not a hardware frame. */
    uint64_t observedAtNs; /* Producer steady-clock callback observation, not device sample time. */
    uint64_t ageNs; /* Meaningful only with AGE_KNOWN; compare to caller's chosen limit. */
    HIGGS_MotionVector3 latestLinear; /* m/s, Skyrim world axes. */
    HIGGS_MotionVector3 latestAngular; /* rad/s, Skyrim world axes. */
    HIGGS_MotionVector3 meanLinear; /* Existing five-entry mean, including restart zero entries. */
    HIGGS_MotionVector3 peakLinear; /* Existing five-entry peak/neighbor throw calculation. */
    float meanSpeed; /* m/s; mean of magnitudes, not magnitude of mean. */
    uint32_t historyObservationCount; /* Consecutive accepted valid observations, 0..5; not hardware samples. */
} HIGGS_MotionHand;

typedef struct HIGGS_MotionSnapshot {
    uint32_t structSize; /* Input buffer capacity; output known v1 size. */
    uint32_t abiVersion; /* Output only; structSize is the sole input field. */
    uint32_t flags;
    uint32_t availableHands; /* Last OBSERVED valid hands; never a freshness guarantee. */
    uint64_t continuityGeneration; /* Private HIGGS lifecycle scope, not a cross-mod world ID. */
    uint64_t processAcceptedObservation; /* Lifetime watermark; not a current-scope sample. */
    uint64_t copiedAtNs; /* Same producer clock used to compute each known age. */
    HIGGS_MotionHand hands[2];
} HIGGS_MotionSnapshot;

typedef struct HIGGS_MotionInterface {
    uint32_t structSize;
    uint32_t abiVersion;
    uint64_t capabilities;
    uint32_t historySize;
    uint32_t reserved;
    HIGGS_MotionResult (HIGGS_MOTION_CALL *CopyVelocitySnapshot)(HIGGS_MotionSnapshot*);
} HIGGS_MotionInterface;
#pragma pack(pop)

/* Resolve HIGGS_QueryMotionInterface from higgs_vr.dll. Exact version only;
   Known v1 sizes/layouts are immutable; later layout changes require a new version.
   tableSize is capacity. Copy requires out.structSize >= sizeof(snapshot).
   Failure leaves ALL output bytes untouched. Success copies only known bytes,
   preserving caller extension bytes. No client/secret/lease or authentication.
   No engine sampling, allocation, pose/palm, compositor-status or actuator service.
   Reset/no-sample hands have no validity bits and zero numeric/time/ticket fields;
   zero vectors without validity are NEVER stationary measurements.
   Choose an age limit explicitly in your consumer and require AGE_KNOWN; there is
   no default timeout. If pose callbacks fall silent, last-observed availability
   persists and age increases. Callers must stop using the table before DLL unload. */
typedef HIGGS_MotionResult (HIGGS_MOTION_CALL *HIGGS_QueryMotionInterfaceFn)(
    uint32_t version, uint32_t tableSize, HIGGS_MotionInterface*);
HIGGS_MotionResult HIGGS_MOTION_CALL HIGGS_QueryMotionInterface(
    uint32_t version, uint32_t tableSize, HIGGS_MotionInterface* outInterface);
#ifdef __cplusplus
}
#endif
#endif
