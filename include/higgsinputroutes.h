#ifndef HIGGS_INPUT_ROUTES_H
#define HIGGS_INPUT_ROUTES_H
#include "higgshandapi.h"
#ifdef __cplusplus
extern "C"
{
#endif
#pragma pack(push, 8)
    enum
    {
        HIGGS_SEQUENCE_HELD = 1,
        HIGGS_SEQUENCE_RELEASED = 2,
        HIGGS_SEQUENCE_CANCELLED = 3
    };
    enum
    {
        HIGGS_ROUTE_NATIVE_GRIP = 1,
        HIGGS_ROUTE_ORDERED_CANDIDATES = 2,
        HIGGS_ROUTE_TERMINALS = 4,
        HIGGS_ROUTE_NATIVE_TRIGGER_GRAB = 8
    };
    typedef struct HIGGS_InputSequence
    {
        uint32_t structSize, phase, physicalHand, button, nativeAction, reason;
        uint64_t epoch, pressInputId, terminalInputId, requestId, executor, correlation, actionId;
    } HIGGS_InputSequence;
    typedef struct HIGGS_InputRoutesInterface
    {
        uint32_t structSize, abiVersion;
        uint64_t capabilities;
        HIGGS_HandStatus(HIGGS_HAND_CALL *SubmitOrderedCandidate)(HIGGS_HandClient, const HIGGS_ActionCandidate *,
                                                                  uint64_t stableTargetKey, uint64_t *requestId);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopySequence)(HIGGS_HandClient, uint64_t pressInputId,
                                                        HIGGS_InputSequence *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CheckSelectedAction)(HIGGS_HandClient, uint64_t requestId,
                                                               uint64_t epoch, uint32_t physicalHands,
                                                               uint64_t *correlation);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyClientLabel)(HIGGS_HandClient, uint64_t *publicLabel);
    } HIGGS_InputRoutesInterface;
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryInputRoutesInterfaceFn)(uint32_t, uint32_t,
                                                                                 HIGGS_InputRoutesInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryInputRoutesInterface(uint32_t, uint32_t,
                                                                     HIGGS_InputRoutesInterface *);
#pragma pack(pop)
/* actionId is the provider's persistent semantic key. Provide a stableTargetKey
   to rank different targets of that action. INI provider/actionId priorities
   use decimal IDs. CheckSelectedAction is available to the selected executor
   and its authorized recognizer while the grant remains valid. Its correlation
   matches the opaque grant field in observed decisions and hand records and
   cannot authorize mutations. CopySequence is an observation: it preserves
   the press/selection through grant release, expires after 256 later terminal
   records and gives no new permission. A native selection or acceptance into HIGGS's
   leeway window does not confirm an object was subsequently grabbed. Cancel has terminalInputId zero. Targeted
   INPUT_RELEASED/CANCELLED events reach the selected executor owner only. Their
   result.inputId is the original press. CopyClientLabel returns a public label
   for this live registration for routing, never a credential or durable key. */
#ifdef __cplusplus
}
#endif
#endif
