#ifndef HIGGS_GRANT_MUTATION_H
#define HIGGS_GRANT_MUTATION_H
#include "higgshandapi.h"
#ifdef __cplusplus
extern "C"
{
#endif
#pragma pack(push, 8)
    typedef struct HIGGS_GrantedMutationRequest
    {
        uint32_t structSize, commitStage; /* COMMITTED or COMPLETED; recorded before fence exit. */
        HIGGS_HandRequestId requestId;
        HIGGS_HandExecutor executor;
        uint64_t epoch;
        HIGGS_HandGrant grant;
    } HIGGS_GrantedMutationRequest;
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_GrantedMutationCallback)(const HIGGS_ActionRequest *,
                                                                             const HIGGS_HandSnapshot *, void *);
    typedef struct HIGGS_GrantMutationInterface
    {
        uint32_t structSize, abiVersion;
        HIGGS_HandStatus(HIGGS_HAND_CALL *WithAuthorizedAction)(HIGGS_HandClient, const HIGGS_GrantedMutationRequest *,
                                                                HIGGS_GrantedMutationCallback, void *,
                                                                HIGGS_ActionResult *);
    } HIGGS_GrantMutationInterface;
#pragma pack(pop)
    /* Exact v1. Runs synchronously only on HIGGS's observed main-maintenance thread.
       Validates assigned executor/request/epoch and every currently owned hand grant
       under the native mutation fence. Callback runs OUTSIDE the broker state mutex;
       no revocation, expiry or world reset may invalidate authority until it returns.
       Same-grant reentrant revoke/remove returns PENDING. Return OK only after the
       domain mutation actually commits (e.g. saved placement CAS); HIGGS records the
       requested stage before releasing the fence. Output is that committed receipt.
       Callback failure/exception cannot promise rollback of its arbitrary effects.
       Inventory transfer/consumption uses TransactionsInterface and its item/permit
       rules; this fence itself creates no item, inventory or physical receipt flag.
       No callback/context is retained. On failure output remains untouched except
       BUFFER_TOO_SMALL sets its required structSize. Stop callers before DLL unload. */
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryGrantMutationInterfaceFn)(uint32_t, uint32_t,
                                                                                   HIGGS_GrantMutationInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryGrantMutationInterface(uint32_t, uint32_t,
                                                                       HIGGS_GrantMutationInterface *);
#ifdef __cplusplus
}
#endif
#endif
