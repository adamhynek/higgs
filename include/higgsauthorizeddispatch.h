#ifndef HIGGS_AUTHORIZED_DISPATCH_H
#define HIGGS_AUTHORIZED_DISPATCH_H
#include "higgshandapi.h"
#define HIGGS_AUTHORIZED_DISPATCH_ABI_VERSION 2u
#ifdef __cplusplus
extern "C"
{
#endif
#pragma pack(push, 8)
    typedef struct HIGGS_AuthorizedDispatchRequest
    {
        uint32_t structSize, reserved;
        HIGGS_HandRequestId requestId;
        HIGGS_HandExecutor executor;
        uint64_t epoch;
        HIGGS_HandGrant grant;
    } HIGGS_AuthorizedDispatchRequest;
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_AuthorizedDispatchCallback)(const HIGGS_ActionRequest *,
                                                                            const HIGGS_HandSnapshot *, void *);
    typedef struct HIGGS_AuthorizedDispatchInterface
    {
        uint32_t structSize, abiVersion;
        HIGGS_HandStatus(HIGGS_HAND_CALL *Invoke)(HIGGS_HandClient, const HIGGS_AuthorizedDispatchRequest *,
                                                HIGGS_AuthorizedDispatchCallback, void *);
    } HIGGS_AuthorizedDispatchInterface;
#pragma pack(pop)
    /* Public snapshot IDs never authorize a mutation. The addressed request retains its own grant. Exact v2, independently negotiated. Invoke runs only on HIGGS's observed
       main-maintenance thread. It checks the selected executor/request/epoch,
       input context and every currently owned, known and available hand resource
       under the native mutation fence, then claims this request once BEFORE
       calling outside the broker state mutex. Duplicate invocation is CONFLICT,
       including reentry and after callback failure/exception. The callback is
       synchronous; no pointer/context is retained. It may queue ordinary child
       physical/transaction requests using the grant. Same-grant revocation,
       removal and ReportResult return PENDING while the callback is entered.
       Callback status is returned (exception becomes INTERNAL_ERROR); invocation
       creates no commit/completion/receipt, rejection or rollback promise. Report
       factual results after return or use the existing child transaction's facts.
       WithAuthorizedAction remains the separate actual-domain-commit fence.
       Requests are copied before callback entry. Stop callers before DLL unload. */
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryAuthorizedDispatchInterfaceFn)(
        uint32_t, uint32_t, HIGGS_AuthorizedDispatchInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryAuthorizedDispatchInterface(
        uint32_t, uint32_t, HIGGS_AuthorizedDispatchInterface *);
#ifdef __cplusplus
}
static_assert(sizeof(HIGGS_AuthorizedDispatchRequest) == 40, "authorized dispatch request ABI");
static_assert(sizeof(HIGGS_AuthorizedDispatchInterface) == 16, "authorized dispatch table ABI (x64)");
#endif
#endif
