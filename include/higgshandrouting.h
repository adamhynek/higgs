#ifndef HIGGS_HAND_ROUTING_H
#define HIGGS_HAND_ROUTING_H
#include "higgshandapi.h"
#ifdef __cplusplus
extern "C"
{
#endif
#define HIGGS_HAND_ROUTING_ABI_VERSION 2u
#pragma pack(push, 8)
    typedef struct HIGGS_HandRoutingInterface
    {
        uint32_t structSize, abiVersion;
        uint64_t capabilities;
        /* Executor owner explicitly authorizes another client's recognizer. Hands is
           a mask, not a grant. Authority is checked again at selection and dispatch. */
        HIGGS_HandStatus(HIGGS_HAND_CALL *AuthorizeRecognizer)(HIGGS_HandClient executorOwner, HIGGS_HandExecutor,
                                                               uint64_t recognizerPublicLabel,
                                                               uint32_t physicalHands);
        HIGGS_HandStatus(HIGGS_HAND_CALL *RevokeRecognizer)(HIGGS_HandClient executorOwner, HIGGS_HandExecutor,
                                                            uint64_t recognizerPublicLabel);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CanRoute)(HIGGS_HandClient recognizerOwner, HIGGS_HandExecutor,
                                                    uint32_t physicalHands);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CancelRecognizedAction)(HIGGS_HandClient recognizerOwner, HIGGS_HandRequestId,
                                                                  uint64_t epoch, uint32_t reason);
    } HIGGS_HandRoutingInterface;
#pragma pack(pop)
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryHandRoutingInterfaceFn)(uint32_t, uint32_t,
                                                                                 HIGGS_HandRoutingInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandRoutingInterface(uint32_t, uint32_t, HIGGS_HandRoutingInterface *);
/* Routed grant/request owner is the assigned executor's client. Recognizer and
   executor may copy results; only executor owner may report/release/use grant.
   A narrower mask requires Revoke then Authorize. Revocation invalidates pending uncommitted routed actions, preserving a
   completed physical hold. It does not release a held item or undo inventory. */
#ifdef __cplusplus
}
#endif
#endif
