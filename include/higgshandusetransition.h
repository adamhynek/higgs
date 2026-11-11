#ifndef HIGGS_HAND_USE_TRANSITION_H
#define HIGGS_HAND_USE_TRANSITION_H
#include "higgshanduse.h"
#ifdef __cplusplus
extern "C" {
#endif
#pragma pack(push,8)
typedef struct HIGGS_HandUseTransitionRequest {
    uint32_t structSize,physicalHands,durationMs,reserved;
    uint64_t epoch,requestId,expectedRevision,expectedHandGenerations[2];
} HIGGS_HandUseTransitionRequest;

typedef struct HIGGS_HandUseTransitionState {
    uint32_t structSize,reserved;
    uint64_t revision;
    HIGGS_HandUseState use;
} HIGGS_HandUseTransitionState;

typedef struct HIGGS_HandUseTransitionInterface {
    uint32_t structSize,abiVersion;
    uint64_t capabilities;
    HIGGS_HandStatus (HIGGS_HAND_CALL *Copy)(HIGGS_HandClient,uint64_t,HIGGS_HandUseTransitionState*);
    HIGGS_HandStatus (HIGGS_HAND_CALL *Resize)(HIGGS_HandClient,const HIGGS_HandUseTransitionRequest*,HIGGS_HandUseTransitionState*);
} HIGGS_HandUseTransitionInterface;
#pragma pack(pop)
/* Atomic live-session footprint changes. Copy supplies the footprint revision;
 * Hand.CopySnapshot supplies generations for all hands in the requested footprint.
 * Successful Resize preserves request/grant/context/kind and the retained hands'
 * authority; it changes the entire footprint or nothing. Every Resize advances
 * revision, including same-footprint renewal. Old HandUse.Renew does not change it.
 * No priority preemption: an added hand must be eligible now. Removed hands must
 * have no live broker child/native/retained/actuator work under this grant: end it
 * and await its actual drainage first. Refusal leaves existing authority intact.
 * Existing held items are never implicitly released. UI permits renew/reduction,
 * but not expansion. Expiry/load/tracking loss cannot be revived by Resize.
 * Separate HandUseProposals supplies nonbutton admission rounds. Resize is not
 * proposal arbitration or a transferable/delegated grant.
 * Exact v1 copied POD, ordinary client/output rules. Capability bit0: resize. */
HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandUseTransitionInterface(uint32_t,uint32_t,HIGGS_HandUseTransitionInterface*);
typedef HIGGS_HandStatus (HIGGS_HAND_CALL *HIGGS_QueryHandUseTransitionInterfaceFn)(uint32_t,uint32_t,HIGGS_HandUseTransitionInterface*);
#ifdef __cplusplus
}
static_assert(sizeof(HIGGS_HandUseTransitionRequest)==56,"HandUse transition request ABI");
static_assert(sizeof(HIGGS_HandUseTransitionState)==136,"HandUse transition state ABI");
static_assert(sizeof(HIGGS_HandUseTransitionInterface)==32,"HandUse transition table ABI (x64)");
#endif
#endif
