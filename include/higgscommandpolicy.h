#ifndef HIGGS_COMMAND_POLICY_H
#define HIGGS_COMMAND_POLICY_H
#include "higgshandapi.h"
#ifdef __cplusplus
extern "C"
{
#endif
    enum
    {
        HIGGS_DRAW_EITHER = 1,
        HIGGS_DRAW_LEFT = 2,
        HIGGS_DRAW_RIGHT = 4,
        HIGGS_DRAW_DUAL_CAST = 8,
        HIGGS_DRAW_ALL_PATHS = 15
    };
    enum
    {
        HIGGS_DRAW_DEFAULT = 0,
        HIGGS_DRAW_ALLOW = 1,
        HIGGS_DRAW_SUPPRESS = 2
    };
#pragma pack(push, 8)
    typedef struct HIGGS_TriggerDrawPolicy
    {
        uint32_t structSize, decision, durationMs, reserved;
        uint64_t epoch;
    } HIGGS_TriggerDrawPolicy;
    typedef struct HIGGS_DefaultCommandInterface
    {
        uint32_t structSize, abiVersion;
        HIGGS_HandStatus(HIGGS_HAND_CALL *AttachDrawBoundary)(HIGGS_HandClient, uint32_t verifiedPaths,
                                                              uint64_t *registration);
        HIGGS_HandStatus(HIGGS_HAND_CALL *DetachDrawBoundary)(HIGGS_HandClient, uint64_t registration);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyDrawCoverage)(uint32_t *paths);
        HIGGS_HandStatus(HIGGS_HAND_CALL *SetTriggerDrawPolicy)(HIGGS_HandClient, const HIGGS_TriggerDrawPolicy *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *EvaluateTriggerDraw)(HIGGS_HandClient, uint64_t registration,
                                                               uint32_t path, uint32_t *decision);
    } HIGGS_DefaultCommandInterface;
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryDefaultCommandInterface(uint32_t, uint32_t,
                                                                        HIGGS_DefaultCommandInterface *);
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryDefaultCommandInterfaceFn)(
        uint32_t, uint32_t, HIGGS_DefaultCommandInterface *);
#pragma pack(pop)
/* Optional engine-side adapter, with all four verified trigger-draw boundaries.
   The adapter calls Evaluate at the semantic command and executes the original
   operation on error or DEFAULT. No controller state is changed. HIGGS has no
   built-in adapter. A registered client sets one epoch-bound, renewable policy
   for 1..60000 ms, or DEFAULT/zero duration to remove it. Provider priority and
   stable provider name select between simultaneous policies. Detaching drops all
   associated policies. A replacement adapter does not inherit them. */
#ifdef __cplusplus
}
#endif
#endif
