#ifndef HIGGSOBSERVATIONS_H
#define HIGGSOBSERVATIONS_H
#include "higgshandapi.h"
#ifdef __cplusplus
extern "C" {
#endif
#pragma pack(push, 8)

/* Additive read-only C ABI; independent of official001 and scoped restrictions. */
#include <stdint.h>
#if defined(_WIN32)
#define HIGGS_MOTION_CALL __cdecl
#else
#define HIGGS_MOTION_CALL
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

#define HIGGS_OBSERVATION_ABI_VERSION 2u
#define HIGGS_OBSERVATION_CAP_MOTION UINT64_C(1)
#define HIGGS_OBSERVATION_CAP_CONTACT UINT64_C(2)
#define HIGGS_OBSERVATION_CAP_RELEASE UINT64_C(4)
#define HIGGS_OBSERVATION_CAP_CONTROLLER_MAPPING UINT64_C(8)
enum { HIGGS_SAMPLE_CONTROLLER=1,HIGGS_SAMPLE_PALM=2,HIGGS_SAMPLE_HELD_BODY=3 };
enum { HIGGS_SPACE_WORLD=1,HIGGS_SPACE_OPENVR_TRACKING=2 };
enum { HIGGS_SAMPLE_AVAILABLE=0,HIGGS_SAMPLE_UNAVAILABLE=1,HIGGS_SAMPLE_INVALID=2,HIGGS_SAMPLE_DISCONNECTED=3 };
enum { HIGGS_SAMPLE_POSE=1,HIGGS_SAMPLE_LINEAR=2,HIGGS_SAMPLE_ANGULAR=4,
       HIGGS_SAMPLE_MAPPING=8,HIGGS_SAMPLE_NO_HARDWARE_TIME=16 };
enum { HIGGS_CONTACT_HAND=1,HIGGS_CONTACT_WEAPON=2,HIGGS_CONTACT_HELD=3 };
enum { HIGGS_CONTACT_ENABLED=1,HIGGS_CONTACT_USED=2,HIGGS_CONTACT_POTENTIAL=4,
       HIGGS_CONTACT_POINT=8,HIGGS_CONTACT_NORMAL=16,HIGGS_CONTACT_LINEAR=32 };
typedef struct HIGGS_MotionSample {
    uint32_t structSize,physicalHand,source,space,flags,itemRefId,status,reserved,deviceIndex,reserved2;
    uint64_t sequence,epoch,observationId,handGeneration,observedAtNs,spaceGeneration,bodyKey,trackingGeneration,captureId;
    float position[3],rotation[9],linearVelocity[3],angularVelocity[3];
    /* Position: world Skyrim units or raw OpenVR metres; linear: metres/s;
     * angular: radians/s. World axes use Skyrim; tracking axes use OpenVR.
     * bodyKey is an observed Havok m_uid scoped by epoch, not a save identity
     * or independently pinned lifetime token. Absent numeric flags mean zeros.
     * Available records have a proper rotation. Other statuses have no pose.
     * observationId names the broker context read at producer entry; controller
     * captureId/spaceGeneration identify their actual paired capture/mapping. */
} HIGGS_MotionSample;

typedef struct HIGGS_TrackingMappingSample {
    uint32_t structSize,flags;
    uint64_t sequence,epoch,spaceGeneration,trackingGeneration,captureId,observedAtNs;
    float rotation[9],translation[3],unitsPerMeter;
    uint32_t reserved;
    /* Maps raw tracking POSITION: worldGameUnits = translation + rotation *
     * rawMetres * unitsPerMeter. World linear velocity is converted to physics
     * metres/s using that scale and the current game/Havok conversion.
     * Pose orientation uses the engine's local-axis conversion separately.
     * Mapping IDs identify captures, not hardware recenter events. */
} HIGGS_TrackingMappingSample;
typedef struct HIGGS_ContactObservation {
    uint32_t structSize,physicalHands,source,flags,itemRefId,targetRefId,targetFormType,reserved;
    uint64_t sequence,epoch,observationId,observedAtNs,probeBodyKey,targetBodyKey;
    float position[3],targetToProbeNormal[3],probeLinearVelocity[3],targetLinearVelocity[3];
    /* Engine enabled contact evidence, not a damage event or an impulse report.
     * Position is world Skyrim units; velocities are world-axis metres/s. */
} HIGGS_ContactObservation;
enum { HIGGS_CONTACT_BODY_UNKNOWN=0, HIGGS_CONTACT_BODY_HAND_PART=1, HIGGS_CONTACT_BODY_HELD_OR_EQUIPPED=2 };
enum { HIGGS_CONTACT_BODY_LIFETIME=1, HIGGS_CONTACT_APPLIED_REVISION=2, HIGGS_CONTACT_LEAF_KEY=4 };
typedef struct HIGGS_ContactEndpoint {
    uint32_t source, fields, observedBodyKey, leafShapeKey;
    uint64_t bodyGeneration, appliedRevision;
} HIGGS_ContactEndpoint;
typedef struct HIGGS_ContactDetails {
    uint32_t structSize, reserved;
    uint64_t epoch, contactSequence;
    HIGGS_ContactEndpoint probe, target;
} HIGGS_ContactDetails;
typedef struct HIGGS_ContactDetailsInterface {
    uint32_t structSize, abiVersion;
    HIGGS_HandStatus (HIGGS_HAND_CALL *Copy)(HIGGS_HandClient,uint64_t epoch,uint64_t contactSequence,HIGGS_ContactDetails*);
} HIGGS_ContactDetailsInterface;
/* Both endpoints are captured at the contact. Lifetime is scoped by source
   and epoch. A leaf key alone is neither an anatomical label nor a complete
   compound shape path. Unknown external endpoints retain their observed UID. */
HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryContactDetailsInterface(uint32_t,uint32_t,HIGGS_ContactDetailsInterface*);
typedef HIGGS_HandStatus (HIGGS_HAND_CALL *HIGGS_QueryContactDetailsInterfaceFn)(uint32_t,uint32_t,HIGGS_ContactDetailsInterface*);
typedef struct HIGGS_ObservationQuery {
    uint32_t structSize,physicalHands,source,space;
    uint64_t epoch,afterSequence,minObservedAtNs;
} HIGGS_ObservationQuery;
typedef struct HIGGS_ReleaseObservation {
    uint32_t structSize,hasMatchingSnapshot;
    HIGGS_HandEvent event;
    HIGGS_HandSnapshot snapshot;
} HIGGS_ReleaseObservation;
typedef struct HIGGS_ObservationAPI {
    uint32_t structSize,abiVersion;
    uint64_t capabilities;
    HIGGS_HandStatus (HIGGS_HAND_CALL *CopyMotionSamples)(HIGGS_HandClient,const HIGGS_ObservationQuery*,HIGGS_MotionSample*,uint32_t,uint32_t*);
    HIGGS_HandStatus (HIGGS_HAND_CALL *CopyContactSnapshot)(HIGGS_HandClient,const HIGGS_ObservationQuery*,HIGGS_ContactObservation*,uint32_t,uint32_t*);
    HIGGS_HandStatus (HIGGS_HAND_CALL *CopyReleaseObservation)(HIGGS_HandClient,uint64_t,HIGGS_ReleaseObservation*);
    HIGGS_HandStatus (HIGGS_HAND_CALL *CopyTrackingMappings)(HIGGS_HandClient,const HIGGS_ObservationQuery*,HIGGS_TrackingMappingSample*,uint32_t,uint32_t*);
} HIGGS_ObservationAPI;
/* Bounded histories:256 motion,512 contacts,64 releases,64 mappings. Queries
 * require exact current query size and nonzero physical-hand mask/epoch.
 * source/space zero select all; contacts/mappings use world space only.
 * Successful copies are ascending sequence. BUFFER_TOO_SMALL changes only
 * count; other failures preserve count/records. No pointers are exported.
 * Capture timestamps use HIGGS steady clock, not hardware acquisition time.
 * No callback-silence freshness or complete manifold enumeration is promised. */
HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryObservationInterface(uint32_t,uint32_t,HIGGS_ObservationAPI*);
typedef HIGGS_HandStatus (HIGGS_HAND_CALL *HIGGS_QueryObservationInterfaceFn)(uint32_t,uint32_t,HIGGS_ObservationAPI*);
#ifdef __cplusplus
static_assert(sizeof(HIGGS_MotionSample)==184,"HIGGS motion record ABI changed");
static_assert(sizeof(HIGGS_TrackingMappingSample)==112,"HIGGS mapping record ABI changed");
static_assert(sizeof(HIGGS_ContactObservation)==128,"HIGGS contact record ABI changed");
static_assert(sizeof(HIGGS_ObservationQuery)==40,"HIGGS observation query ABI changed");
static_assert(sizeof(HIGGS_ObservationAPI)==48,"HIGGS observation table ABI changed (x64)");
#endif

#pragma pack(pop)
#ifdef __cplusplus
}
#endif
#endif
