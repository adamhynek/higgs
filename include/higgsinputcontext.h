#ifndef HIGGS_INPUT_CONTEXT_H
#define HIGGS_INPUT_CONTEXT_H
#include "higgshandapi.h"
#ifdef __cplusplus
extern "C" {
#endif
#pragma pack(push, 8)
#define HIGGS_INPUT_CONTEXT_ABI_VERSION 1u
enum HIGGS_InputContextKind { HIGGS_CONTEXT_GAMEPLAY = 1, HIGGS_CONTEXT_UI = 2 };
enum HIGGS_InputContextFlags { HIGGS_CONTEXT_DRAINING = 1u, HIGGS_CONTEXT_NO_REPLAY = 2u };
typedef struct HIGGS_InputContextHand {
    uint64_t frameId, drainPressed, drainTouched;
} HIGGS_InputContextHand;
typedef struct HIGGS_InputContextState {
    uint32_t structSize, kind, activeMenus, flags;
    uint64_t epoch, generation, changedAtNs;
    HIGGS_InputContextHand hands[2];
} HIGGS_InputContextState;
typedef struct HIGGS_InputContextFrame {
    uint32_t structSize, physicalHand, kind, flags;
    uint64_t epoch, frameId, generation, drainPressed, drainTouched;
} HIGGS_InputContextFrame;
typedef void(HIGGS_HAND_CALL *HIGGS_InputContextCallback)(const HIGGS_InputContextState *previous,
    const HIGGS_InputContextState *current, void *context);
typedef struct HIGGS_InputContextInterface {
    uint32_t structSize, abiVersion;
    uint64_t capabilities; /* bit0: menu handoff, bit1: frame association, bit2: removable events */
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopyContext)(HIGGS_HandClient, HIGGS_InputContextState *);
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopyFrameContext)(HIGGS_HandClient, uint64_t frameId, HIGGS_InputContextFrame *);
    HIGGS_HandStatus(HIGGS_HAND_CALL *Subscribe)(HIGGS_HandClient, HIGGS_InputContextCallback, void *, HIGGS_HandRegistration *);
} HIGGS_InputContextInterface;
typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryInputContextInterfaceFn)(uint32_t, uint32_t, HIGGS_InputContextInterface *);
HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryInputContextInterface(uint32_t, uint32_t, HIGGS_InputContextInterface *);
#pragma pack(pop)
/* Native menu events are authoritative. UI is an input context, not a pause claim.
 * Raw observations continue without gameplay authority. Existing physical use and
 * real-time expiry continue; new gameplay admission is suspended. Held controls
 * drain through release in both directions, never replay. Frame associations are
 * bounded to the last128 controller samples. Events run serially on the transition
 * thread outside the state mutex. Remove/drain via HandInterface.RemoveRegistration.
 * This describes returned controller/HIGGS routing, not all engine input paths. */
#ifdef __cplusplus
}
static_assert(sizeof(HIGGS_InputContextState) == 88, "context state layout");
static_assert(sizeof(HIGGS_InputContextFrame) == 56, "context frame layout");
static_assert(sizeof(HIGGS_InputContextInterface) == 40, "context table layout");
#endif
#endif
