#include "hand_abi_boundary.h"
#include "hand_broker.h"
#include "hand_transactions.h"
#include <cstring>
namespace
{
using namespace HiggsHands;


HIGGS_HandStatus HIGGS_HAND_CALL Declare(uint64_t owner, const HIGGS_ItemDeclaration *d, HIGGS_ItemIdentity *out)
{
    if (!d || d->structSize < sizeof(*d))
        return HIGGS_HAND_INVALID_ARGUMENT;
    auto s = HiggsAbi::CheckSized(out);
    if (s)
        return s;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return Transactions().Declare(owner, *d, value);
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Held(uint64_t owner, uint32_t hand, HIGGS_ItemIdentity *out)
{
    auto s = HiggsAbi::CheckSized(out);
    if (s)
        return s;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return Transactions().Held(owner, hand, value);
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Executor(uint64_t owner, HIGGS_TransferExecutorCallback cb, void *context,
                                          uint64_t *out)
{
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return Service().TransferExecutor(owner, cb, context, value);
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Request(uint64_t owner, const HIGGS_TransferRequest *r, uint64_t *out)
{
    if (!r || r->structSize < sizeof(*r) || !out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return Transactions().Request(owner, *r, value, Now());
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Begin(uint64_t owner, uint64_t request, uint64_t epoch, uint64_t *out)
{
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return Transactions().Begin(owner, request, epoch, value);
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Report(uint64_t owner, const HIGGS_TransferReceipt *r)
{
    if (!r || r->structSize < sizeof(*r))
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::Guard([&] { return Transactions().Report(owner, *r); });
}
HIGGS_HandStatus HIGGS_HAND_CALL Copy(uint64_t owner, uint64_t request, HIGGS_TransferState *out)
{
    auto s = HiggsAbi::CheckSized(out);
    if (s)
        return s;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return Transactions().Copy(owner, request, value);
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Cancel(uint64_t owner, uint64_t request, uint64_t epoch)
{
    return HiggsAbi::Guard([&] { return Transactions().Cancel(owner, request, epoch); });
}
} // namespace
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryTransactionsInterface(uint32_t version, uint32_t bytes,
                                                                             HIGGS_TransactionsInterface *out)
{
    if (version != HIGGS_TRANSACTIONS_VERSION)
        return HIGGS_HAND_UNSUPPORTED_VERSION;
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (bytes < sizeof(*out))
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    const HIGGS_TransactionsInterface table{sizeof(table),
                                            version,
                                            HIGGS_TX_CAP_ITEM_IDENTITIES | HIGGS_TX_CAP_TRANSFERS |
                                                HIGGS_TX_CAP_CONSUMPTION | HIGGS_TX_CAP_MUTATION_PERMITS |
                                                HIGGS_TX_CAP_GAME_EQUIPMENT,
                                            Declare,
                                            Held,
                                            Executor,
                                            Request,
                                            Begin,
                                            Report,
                                            Copy,
                                            Cancel};
    std::memcpy(out, &table, sizeof(table));
    return HIGGS_HAND_OK;
}
