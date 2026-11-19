#include "hand_abi_boundary.h"
#include "hand_broker.h"
#include "hand_input_extended.h"
#include <cstring>
namespace
{

const HIGGS_ControlInfo catalogue[]{
    {0, 1, "System"},     {1, 11, "ApplicationMenu"}, {2, 15, "Grip"},           {3, 11, "DPadLeft"},
    {4, 11, "DPadUp"},    {5, 11, "DPadRight"},       {6, 11, "DPadDown"},       {7, 11, "A"},
    {31, 1, "Proximity"}, {32, 11, "Axis0/Touchpad"}, {33, 11, "Axis1/Trigger"}, {34, 11, "Axis2"},
    {35, 11, "Axis3"},    {36, 11, "Axis4"}};
HIGGS_HandStatus HIGGS_HAND_CALL Catalog(HIGGS_ControlInfo *out, uint32_t capacity, uint32_t *count)
{
    if (!count || (capacity && !out) || capacity > 64)
        return HIGGS_HAND_INVALID_ARGUMENT;
    constexpr uint32_t needed = sizeof(catalogue) / sizeof(catalogue[0]);
    if (capacity < needed)
    {
        *count = needed;
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    }
    for (uint32_t i = 0; i < needed; ++i)
        out[i] = catalogue[i];
    *count = needed;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus HIGGS_HAND_CALL Snapshot(uint64_t client, HIGGS_InputSnapshot *out)
{
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (out->structSize < sizeof(*out))
    {
        out->structSize = sizeof(*out);
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    }
    return HiggsAbi::Guard([&]() -> HIGGS_HandStatus {
        if (!HiggsHands::Service().ClientValid(client))
            return HIGGS_HAND_INVALID_CLIENT;
        const auto epoch = HiggsHands::Service().Epoch();
        if (!epoch)
            return HIGGS_HAND_NOT_READY;
        HiggsHands::InputFrames().Epoch(epoch);
        HIGGS_InputSnapshot value{};
        const auto status = HiggsHands::InputFrames().Copy(epoch, value);
        if (!status)
            *out = value;
        return status;
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Frames(uint64_t client, const HIGGS_InputFrameQuery *query, HIGGS_ControllerFrame *out,
                                        uint32_t capacity, uint32_t *count)
{
    if (!query || !count)
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::Guard([&]() -> HIGGS_HandStatus {
        if (!HiggsHands::Service().ClientValid(client))
            return HIGGS_HAND_INVALID_CLIENT;
        const auto epoch = HiggsHands::Service().Epoch();
        if (!epoch)
            return HIGGS_HAND_NOT_READY;
        HiggsHands::InputFrames().Epoch(epoch);
        if (query->epoch != epoch)
            return HIGGS_HAND_STALE;
        return HiggsHands::InputFrames().History(*query, out, capacity, *count);
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Register(uint64_t client, const HIGGS_InputRecognizerSpec *spec, uint64_t *out)
{
    if (!spec || !out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& token) {
        return HiggsHands::Service().InputRecognizer(client, *spec, token);
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Submit(uint64_t client, const HIGGS_InputCandidate *spec, uint64_t *out)
{
    if (!spec || !out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& request) {
        return HiggsHands::Service().SubmitInputCandidate(client, *spec, request, HiggsHands::Now());
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Recognition(uint64_t client, uint64_t request, HIGGS_InputRecognition *out)
{
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (out->structSize < sizeof(*out))
    {
        out->structSize = sizeof(*out);
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    }
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return HiggsHands::Service().CopyInputRecognition(client, request, value);
    });
}
} // namespace
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryInputInterface(uint32_t version, uint32_t bytes,
                                                                      HIGGS_InputInterface *out)
{
    if (version != HIGGS_INPUT_ABI_VERSION)
        return HIGGS_HAND_UNSUPPORTED_VERSION;
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (bytes < sizeof(*out))
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    const HIGGS_InputInterface table{
        sizeof(table), HIGGS_INPUT_ABI_VERSION, 31, Catalog, Snapshot, Frames, Register, Submit, Recognition};
    std::memcpy(out, &table, sizeof(table));
    return HIGGS_HAND_OK;
}
