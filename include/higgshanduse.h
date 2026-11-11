#ifndef HIGGS_HAND_USE_H
#define HIGGS_HAND_USE_H
#include "higgshandapi.h"
#ifdef __cplusplus
extern "C"
{
#endif
    enum
    {
        HIGGS_USE_HAND_GOALS = 1,
        HIGGS_USE_GAME_EQUIPMENT = 2,
        HIGGS_USE_HELD_OBJECT = 3,
        HIGGS_USE_SECONDARY_SUPPORT = 4,
        HIGGS_USE_WORLD_SUPPORT = 5,
        HIGGS_USE_CONSTRAINT = 6,
        HIGGS_USE_CONSUMPTION = 7
    };
#pragma pack(push, 8)
    typedef struct HIGGS_HandUseRequest
    {
        uint32_t structSize, kind, physicalHands, durationMs;
        uint64_t epoch, contextId, contextGeneration;
        HIGGS_HandExecutor executor;
    } HIGGS_HandUseRequest;
    typedef struct HIGGS_HandUseState
    {
        uint32_t structSize, kind, active, reserved;
        uint64_t epoch, expiresAtNs;
        HIGGS_ActionRequest authority;
    } HIGGS_HandUseState;
    typedef struct HIGGS_HandUseInterface
    {
        uint32_t structSize, abiVersion;
        uint64_t capabilities;
        HIGGS_HandStatus(HIGGS_HAND_CALL *RequestHandUse)(HIGGS_HandClient, const HIGGS_HandUseRequest *,
                                                          HIGGS_HandUseState *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *RenewHandUse)(HIGGS_HandClient, HIGGS_HandRequestId, uint64_t, uint32_t,
                                                        HIGGS_HandUseState *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyHandUse)(HIGGS_HandClient, HIGGS_HandRequestId, HIGGS_HandUseState *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *EndHandUse)(HIGGS_HandClient, HIGGS_HandRequestId, uint64_t);
    } HIGGS_HandUseInterface;
#pragma pack(pop)
    /* Externally initiated continuous uses (activation, contact or domain events)
       require no fabricated controller press. This finite catalogue authorizes hand
       resources and reserves all named hands atomically; acceptance is NOT physical
       actuation. Apply supported goals/grips/constraints through their own receipts.
       The assigned generic executor is notified once in HIGGS main maintenance with
       inputId=0, actionId=kind and copied context identity. Use GrantMutationInterface
       for a fenced domain-state commit. Renew validates current ownership and cannot
       revive expired, reset, unregistered or tracking-invalid grants. Ending removes
       this use's grants/effects, never implicitly releases an already held item.
       This is a reservation primitive, not permission for a custom physical driver.
       Durations1..60000ms, exact copied v1, existing shared-client/registration rules. */
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryHandUseInterfaceFn)(uint32_t, uint32_t,
                                                                             HIGGS_HandUseInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandUseInterface(uint32_t, uint32_t, HIGGS_HandUseInterface *);
#ifdef __cplusplus
}
#endif
#endif
