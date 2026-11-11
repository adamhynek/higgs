#include "hand_abi_boundary.h"
#include "hand_broker.h"
#include "higgshandapi.h"
#include <cstring>
#include <type_traits>
namespace
{
using namespace HiggsHands;


HIGGS_HandStatus HIGGS_HAND_CALL Register(const HIGGS_HandClientInfo *p, uint64_t *out)
{
    if (!p || !out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& id) {
        return Service().Register(*p, id);
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Unregister(uint64_t owner)
{
    return HiggsAbi::Guard([&] { return Service().Unregister(owner); });
}
HIGGS_HandStatus HIGGS_HAND_CALL Copy(uint64_t owner, HIGGS_HandSnapshot *out)
{
    auto s = HiggsAbi::CheckSized(out);
    if (s)
        return s;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return Service().ObservedSnapshot(owner, value, Now());
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Recognizer(uint64_t o, HIGGS_RecognizerCallback cb, void *ctx, uint64_t *out)
{
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return Service().Recognizer(o, cb, ctx, value);
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Executor(uint64_t o, HIGGS_ExecutorCallback cb, void *ctx, uint64_t *out)
{
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return Service().Executor(o, cb, ctx, value);
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Events(uint64_t o, HIGGS_HandEventCallback cb, void *ctx, uint64_t *out)
{
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return Service().Events(o, cb, ctx, value);
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Remove(uint64_t o, uint64_t id)
{
    return HiggsAbi::Guard([&] { return Service().Remove(o, id); });
}
HIGGS_HandStatus HIGGS_HAND_CALL Submit(uint64_t o, const HIGGS_ActionCandidate *p, uint64_t *out)
{
    if (!p || !out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return Service().Submit(o, *p, value, Now());
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Report(uint64_t o, const HIGGS_ActionResult *p)
{
    if (!p || p->structSize < sizeof(*p))
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::Guard([&] { return Service().Report(o, *p); });
}
HIGGS_HandStatus HIGGS_HAND_CALL Result(uint64_t o, uint64_t id, HIGGS_ActionResult *out)
{
    auto s = HiggsAbi::CheckSized(out);
    if (s)
        return s;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return Service().Result(o, id, value);
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Decision(uint64_t o, uint64_t id, HIGGS_InputDecision *out)
{
    auto s = HiggsAbi::CheckSized(out);
    if (s)
        return s;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return Service().ObservedDecision(o, id, value);
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Release(uint64_t o, uint64_t id)
{
    return HiggsAbi::Guard([&] { return Service().Release(o, id); });
}
HIGGS_HandStatus HIGGS_HAND_CALL Physical(uint64_t o, const HIGGS_PhysicalRequest *p, uint64_t *out)
{
    if (!p || !out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return Service().Physical(o, *p, value, Now());
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Authorize(uint64_t o, uint64_t e, uint64_t r, uint32_t h)
{
    return HiggsAbi::Guard([&] {
        const auto recognizer=Service().ClientForLabel(r);
        return recognizer ? Service().Authorize(o,e,recognizer,h) : HIGGS_HandStatus(HIGGS_HAND_INVALID_CLIENT);
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Revoke(uint64_t o, uint64_t e, uint64_t r)
{
    return HiggsAbi::Guard([&] {
        const auto recognizer=Service().ClientForLabel(r);
        return recognizer ? Service().Revoke(o,e,recognizer) : HIGGS_HandStatus(HIGGS_HAND_INVALID_CLIENT);
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL CanRoute(uint64_t r, uint64_t e, uint32_t h)
{
    return HiggsAbi::Guard([&] { return Service().CanRoute(r, e, h); });
}
HIGGS_HandStatus HIGGS_HAND_CALL CancelRecognized(uint64_t r, uint64_t request, uint64_t epoch, uint32_t reason)
{
    return HiggsAbi::Guard([&] { return Service().CancelRecognized(r, request, epoch, reason); });
}
HIGGS_HandStatus HIGGS_HAND_CALL Operation(uint64_t owner, const HIGGS_HandOperation *p, uint64_t *out)
{
    if (!p || !out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return Service().Operation(owner, *p, value, Now());
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL EndOperation(uint64_t owner, uint64_t request, uint64_t epoch)
{
    return HiggsAbi::Guard([&] { return Service().EndOperation(owner, request, epoch); });
}
HIGGS_HandStatus HIGGS_HAND_CALL CopyOperation(uint64_t owner, uint64_t request, HIGGS_HandOperationState *out)
{
    auto s = HiggsAbi::CheckSized(out);
    if (s)
        return s;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return Service().CopyOperation(owner, request, value, Now());
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Authorized(uint64_t owner, const HIGGS_GrantedMutationRequest *r,
                                            HIGGS_GrantedMutationCallback callback, void *context,
                                            HIGGS_ActionResult *out)
{
    if (!r || r->structSize < sizeof(*r))
        return HIGGS_HAND_INVALID_ARGUMENT;
    auto status = HiggsAbi::CheckSized(out);
    if (status)
        return status;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return Service().WithAuthorizedAction(owner, *r, callback, context, value);
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL InvokeAuthorized(uint64_t owner, const HIGGS_AuthorizedDispatchRequest *r,
                                                 HIGGS_AuthorizedDispatchCallback callback, void *context)
{
    if (!r || r->structSize != sizeof(*r))
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::Guard([&] { return Service().InvokeAuthorized(owner, *r, callback, context); });
}
} // namespace
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandRoutingInterface(uint32_t version, uint32_t bytes,
                                                                            HIGGS_HandRoutingInterface *out)
{
    if (version != HIGGS_HAND_ROUTING_ABI_VERSION)
        return HIGGS_HAND_UNSUPPORTED_VERSION;
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (bytes < sizeof(*out))
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    const HIGGS_HandRoutingInterface table{
        sizeof(table), HIGGS_HAND_ROUTING_ABI_VERSION, 1, Authorize, Revoke, CanRoute, CancelRecognized};
    std::memcpy(out, &table, sizeof(table));
    return HIGGS_HAND_OK;
}
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandOperationsInterface(uint32_t version, uint32_t bytes,
                                                                               HIGGS_HandOperationsInterface *out)
{
    if (version != HIGGS_HAND_OPERATIONS_ABI_VERSION)
        return HIGGS_HAND_UNSUPPORTED_VERSION;
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (bytes < sizeof(*out))
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    const HIGGS_HandOperationsInterface table{sizeof(table),
                                              HIGGS_HAND_OPERATIONS_ABI_VERSION,
                                              HIGGS_HAND_OP_CAP_STATIC_WORLD_SUPPORT |
                                                  HIGGS_HAND_OP_CAP_PHYSICAL_HAND_TARGET |
                                                  HIGGS_HAND_OP_CAP_GRIP_TRANSFORM | HIGGS_HAND_OP_CAP_DRIVE_CAP |
                                                  HIGGS_HAND_OP_CAP_HAPTIC | HIGGS_HAND_OP_CAP_HAND_GEOMETRY | HIGGS_HAND_OP_CAP_SECONDARY_GRIP_TRANSFORM,
                                              Operation,
                                              EndOperation,
                                              CopyOperation};
    std::memcpy(out, &table, sizeof(table));
    return HIGGS_HAND_OK;
}
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryGrantMutationInterface(uint32_t version, uint32_t bytes,
                                                                              HIGGS_GrantMutationInterface *out)
{
    if (version != HIGGS_GRANT_MUTATION_ABI_VERSION)
        return HIGGS_HAND_UNSUPPORTED_VERSION;
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (bytes < sizeof(*out))
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    const HIGGS_GrantMutationInterface table{sizeof(table), HIGGS_GRANT_MUTATION_ABI_VERSION, Authorized};
    std::memcpy(out, &table, sizeof(table));
    return HIGGS_HAND_OK;
}
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryAuthorizedDispatchInterface(
    uint32_t version, uint32_t bytes, HIGGS_AuthorizedDispatchInterface *out)
{
    if (version != HIGGS_AUTHORIZED_DISPATCH_ABI_VERSION)
        return HIGGS_HAND_UNSUPPORTED_VERSION;
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (bytes < sizeof(*out))
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    const HIGGS_AuthorizedDispatchInterface table{sizeof(table), HIGGS_AUTHORIZED_DISPATCH_ABI_VERSION, InvokeAuthorized};
    std::memcpy(out, &table, sizeof(table));
    return HIGGS_HAND_OK;
}
static_assert(std::is_standard_layout<HIGGS_HandSnapshot>::value && std::is_trivially_copyable<HIGGS_HandInput>::value,
              "Copied C records");
namespace {
HIGGS_HandStatus HIGGS_HAND_CALL Ordered(uint64_t owner, const HIGGS_ActionCandidate *p, uint64_t target, uint64_t *out)
{
    if (!p || !out) return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::Guard([&]{ return Service().Submit(owner,*p,*out,Now(),target); });
}
HIGGS_HandStatus HIGGS_HAND_CALL Sequence(uint64_t owner, uint64_t press, HIGGS_InputSequence *out)
{
    const auto status = HiggsAbi::CheckSized(out);
    if (status) return status;
    return HiggsAbi::Guard([&]{ return Service().CopySequence(owner,press,*out); });
}
HIGGS_HandStatus HIGGS_HAND_CALL CurrentAction(uint64_t owner,uint64_t id,uint64_t epoch,uint32_t hands,uint64_t *out)
{
    if (!out) return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::Guard([&]{ return Service().CheckSelectedAction(owner,id,epoch,hands,*out,Now()); });
}
HIGGS_HandStatus HIGGS_HAND_CALL Label(uint64_t owner,uint64_t *out)
{
    if(!out)return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::Guard([&]() -> HIGGS_HandStatus {
        const auto label=Service().ClientLabel(owner);
        if(!label)return HIGGS_HAND_INVALID_CLIENT;
        *out=label;return HIGGS_HAND_OK;
    });
}
}
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryInputRoutesInterface(uint32_t version,uint32_t bytes,HIGGS_InputRoutesInterface *out)
{
    if (version != 1) return HIGGS_HAND_UNSUPPORTED_VERSION;
    if (!out) return HIGGS_HAND_INVALID_ARGUMENT;
    if (bytes < sizeof(*out)) return HIGGS_HAND_BUFFER_TOO_SMALL;
    *out = {sizeof(*out),1,HIGGS_ROUTE_NATIVE_GRIP|HIGGS_ROUTE_ORDERED_CANDIDATES|HIGGS_ROUTE_TERMINALS|HIGGS_ROUTE_NATIVE_TRIGGER_GRAB,Ordered,Sequence,CurrentAction,Label};
    return HIGGS_HAND_OK;
}
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandInterface(uint32_t version, uint32_t bytes,
                                                                     HIGGS_HandInterface *out)
{
    if (version != HIGGS_HAND_ABI_VERSION)
        return HIGGS_HAND_UNSUPPORTED_VERSION;
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (bytes < sizeof(*out))
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    const HIGGS_HandInterface table{sizeof(table),
                                    HIGGS_HAND_ABI_VERSION,
                                    HIGGS_HAND_CAP_SNAPSHOT | HIGGS_HAND_CAP_INPUT_BROKER |
                                        HIGGS_HAND_CAP_OWNED_GRANTS | HIGGS_HAND_CAP_EXECUTOR_RECEIPTS |
                                        HIGGS_HAND_CAP_PHYSICAL_REQUESTS | HIGGS_HAND_CAP_INTERACTION_EVENTS,
                                    Register,
                                    Unregister,
                                    Copy,
                                    Recognizer,
                                    Executor,
                                    Events,
                                    Remove,
                                    Submit,
                                    Report,
                                    Result,
                                    Decision,
                                    Release,
                                    Physical};
    std::memcpy(out, &table, sizeof(table));
    return HIGGS_HAND_OK;
}
