#include "hand_abi_boundary.h"
#include "hand_broker.h"
#include "hand_physical_policy.h"
#include <cstring>
namespace
{

HIGGS_HandStatus HIGGS_HAND_CALL Request(uint64_t owner, const HIGGS_PhysicalPolicyRequest *request, uint64_t *out)
{
    if (!request || request->structSize < sizeof(*request) || !out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& id) {
        return HiggsHands::PhysicalPolicies().Request(owner, *request, id, HiggsHands::Now());
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Update(uint64_t owner, uint64_t id, uint64_t revision,
                                        const HIGGS_PhysicalPolicyRequest *request)
{
    if (!request || request->structSize < sizeof(*request))
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::Guard(
        [&] { return HiggsHands::PhysicalPolicies().Update(owner, id, revision, *request, HiggsHands::Now()); });
}
HIGGS_HandStatus HIGGS_HAND_CALL End(uint64_t owner, uint64_t id, uint64_t epoch)
{
    return HiggsAbi::Guard([&] { return HiggsHands::PhysicalPolicies().End(owner, id, epoch); });
}
HIGGS_HandStatus HIGGS_HAND_CALL Copy(uint64_t owner, uint64_t id, HIGGS_PhysicalPolicyState *out)
{
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (out->structSize < sizeof(*out))
    {
        out->structSize = sizeof(*out);
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    }
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return HiggsHands::PhysicalPolicies().Copy(owner, id, value);
    });
}
} // namespace
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryPhysicalPolicyInterface(uint32_t version, uint32_t bytes,
                                                                               HIGGS_PhysicalPolicyInterface *out)
{
    if (version != HIGGS_PHYSICAL_POLICY_ABI_VERSION)
        return HIGGS_HAND_UNSUPPORTED_VERSION;
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (bytes < sizeof(*out))
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    const HIGGS_PhysicalPolicyInterface table{
        sizeof(table), HIGGS_PHYSICAL_POLICY_ABI_VERSION, HiggsHands::PhysicalPolicies().Capabilities(), Request, Update, End, Copy};
    std::memcpy(out, &table, sizeof(table));
    return HIGGS_HAND_OK;
}
