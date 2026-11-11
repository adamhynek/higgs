#ifndef HIGGS_OBSERVATIONS_H
#define HIGGS_OBSERVATIONS_H
#include "higgshandapi.h"
#define HIGGS_OBSERVATION_ABI_VERSION 2u
#define HIGGS_OBSERVATION_CAP_MOTION UINT64_C(1)
#define HIGGS_OBSERVATION_CAP_CONTACT UINT64_C(2)
#define HIGGS_OBSERVATION_CAP_RELEASE UINT64_C(4)
#define HIGGS_OBSERVATION_CAP_CONTROLLER_MAPPING UINT64_C(8)
#ifdef __cplusplus
extern "C" {
#endif
enum { HIGGS_SAMPLE_CONTROLLER=1,HIGGS_SAMPLE_PALM=2,HIGGS_SAMPLE_HELD_BODY=3 };
enum { HIGGS_SPACE_WORLD=1,HIGGS_SPACE_OPENVR_TRACKING=2 };
enum { HIGGS_SAMPLE_AVAILABLE=0,HIGGS_SAMPLE_UNAVAILABLE=1,HIGGS_SAMPLE_INVALID=2,HIGGS_SAMPLE_DISCONNECTED=3 };
enum { HIGGS_SAMPLE_POSE=1,HIGGS_SAMPLE_LINEAR=2,HIGGS_SAMPLE_ANGULAR=4,
       HIGGS_SAMPLE_MAPPING=8,HIGGS_SAMPLE_NO_HARDWARE_TIME=16 };
enum { HIGGS_CONTACT_HAND=1,HIGGS_CONTACT_WEAPON=2,HIGGS_CONTACT_HELD=3 };
enum { HIGGS_CONTACT_ENABLED=1,HIGGS_CONTACT_USED=2,HIGGS_CONTACT_POTENTIAL=4,
       HIGGS_CONTACT_POINT=8,HIGGS_CONTACT_NORMAL=16,HIGGS_CONTACT_LINEAR=32 };
#pragma pack(push,8)
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
#pragma pack(pop)
#ifdef __cplusplus
}
static_assert(sizeof(HIGGS_MotionSample)==184,"HIGGS motion record ABI changed");
static_assert(sizeof(HIGGS_TrackingMappingSample)==112,"HIGGS mapping record ABI changed");
static_assert(sizeof(HIGGS_ContactObservation)==128,"HIGGS contact record ABI changed");
static_assert(sizeof(HIGGS_ObservationQuery)==40,"HIGGS observation query ABI changed");
static_assert(sizeof(HIGGS_ObservationAPI)==48,"HIGGS observation table ABI changed (x64)");
#endif
#endif
