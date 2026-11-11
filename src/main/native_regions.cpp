#include "hand_abi_boundary.h"
#include "hand_broker.h"
#include "native_regions.h"
#include <cmath>
#include <cstring>
#include <type_traits>
namespace HiggsHands {
bool NativeRegionGeometryValid(const HIGGS_NativeRegionFrame& frame) {
    if (frame.structSize != sizeof(frame) || frame.reserved ||
        (frame.physicalHand != HIGGS_HAND_LEFT && frame.physicalHand != HIGGS_HAND_RIGHT) ||
        frame.flags != HIGGS_REGION_SOURCE_VALID || !std::isfinite(frame.hmdScale) || frame.hmdScale <= 0)
        return false;
    for (auto value : frame.hmdPosition) if (!std::isfinite(value)) return false;
    for (auto value : frame.hmdRotation) if (!std::isfinite(value)) return false;
    for (auto value : frame.handPosition) if (!std::isfinite(value)) return false;
    for (auto value : frame.palmPosition) if (!std::isfinite(value)) return false;
    for (const auto& sphere : frame.spheres) {
        for (auto value : sphere.worldCenter) if (!std::isfinite(value)) return false;
        if (!std::isfinite(sphere.worldRadius)) return false;
    }
    return true;
}
uint32_t NativeRegionFallbackMask(const HIGGS_NativeRegionFrame& frame) {
    if (!NativeRegionGeometryValid(frame)) return 0;
    uint32_t inside = 0;
    for (uint32_t i = 0; i < HIGGS_NATIVE_REGION_COUNT; ++i) {
        const auto* probe = i ? frame.handPosition : frame.palmPosition;
        const auto& sphere = frame.spheres[i];
        // Match the original float VectorLength < radius, including zero or
        // negative configured radii disabling a region rather than failing all.
        const float x = probe[0] - sphere.worldCenter[0];
        const float y = probe[1] - sphere.worldCenter[1];
        const float z = probe[2] - sphere.worldCenter[2];
        if (std::sqrt(x*x + y*y + z*z) < sphere.worldRadius) inside |= 1u << i;
    }
    return inside;
}
}
namespace {

HIGGS_HandStatus HIGGS_HAND_CALL Register(uint64_t owner, HIGGS_NativeRegionDetector callback,
                                        void* context, uint64_t* out) {
    if (!out) return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return HiggsHands::Service().NativeRegionDetector(
        owner, callback, context, value);
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Remove(uint64_t owner, uint64_t registration) {
    return HiggsAbi::Guard([&] { return HiggsHands::Service().RemoveNativeRegionDetector(owner, registration); });
}
HIGGS_HandStatus HIGGS_HAND_CALL Copy(uint64_t owner, uint32_t hand, HIGGS_NativeRegionFrame* out) {
    if (!out) return HIGGS_HAND_INVALID_ARGUMENT;
    if (out->structSize < sizeof(*out)) { out->structSize = sizeof(*out); return HIGGS_HAND_BUFFER_TOO_SMALL; }
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return HiggsHands::Service().CopyNativeRegionFrame(owner, hand, value);
    });
}
}
extern "C" const HIGGS_NativeRegionsInterface* HIGGS_HAND_CALL
HIGGS_QueryNativeRegionsInterface(uint32_t version, uint32_t size) {
    static const HIGGS_NativeRegionsInterface table{sizeof(table), HIGGS_NATIVE_REGIONS_ABI_VERSION,
        HIGGS_NATIVE_REGION_CAP_DETECTOR | HIGGS_NATIVE_REGION_CAP_COPIED_FRAME,
        Register, Remove, Copy};
    return version == HIGGS_NATIVE_REGIONS_ABI_VERSION && size >= sizeof(table) ? &table : nullptr;
}
static_assert(sizeof(HIGGS_NativeRegionFrame) == 184);
static_assert(sizeof(HIGGS_NativeRegionResult) == 24);
static_assert(sizeof(HIGGS_NativeRegionsInterface) == 40);
static_assert(std::is_trivially_copyable_v<HIGGS_NativeRegionFrame>);
