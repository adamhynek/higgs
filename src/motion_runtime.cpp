#include "motion_runtime.h"
#include <cstring>
#include <type_traits>

static_assert(sizeof(HIGGS_MotionVector3) == 12 && sizeof(HIGGS_MotionHand) == 96, "Motion hand ABI");
static_assert(sizeof(HIGGS_MotionSnapshot) == 232 && sizeof(HIGGS_MotionInterface) == 32, "Motion pair/table ABI (x64)");
static_assert(std::is_trivially_copyable<HIGGS_MotionSnapshot>::value, "Copied POD only");

namespace HiggsMotion {
    namespace {
        std::uint32_t PublicStatus(Status status) {
            switch (status) {
            case Status::Valid: return HIGGS_MOTION_VALID;
            case Status::InvalidPose: return HIGGS_MOTION_INVALID_POSE;
            case Status::Disconnected: return HIGGS_MOTION_DISCONNECTED;
            case Status::InvalidConversion: return HIGGS_MOTION_INVALID_CONVERSION;
            case Status::NonFinite: return HIGGS_MOTION_NONFINITE;
            case Status::WorldUnavailable: return HIGGS_MOTION_WORLD_UNAVAILABLE;
            default: return HIGGS_MOTION_UNAVAILABLE;
            }
        }
        HIGGS_MotionVector3 Vector(const Vector3& v) { return {v.x, v.y, v.z}; }
    }
    HIGGS_MotionSnapshot VelocitySnapshot(const CopiedTracking& source) {
        HIGGS_MotionSnapshot out{};
        out.structSize = sizeof(out); out.abiVersion = HIGGS_MOTION_ABI_VERSION;
        out.continuityGeneration = source.generation; out.processAcceptedObservation = source.lastAcceptedObservation;
        out.copiedAtNs = source.copiedAtNs;
        if (source.worldObserved) out.flags |= HIGGS_MOTION_WORLD_OBSERVED;
        if (source.loading) out.flags |= HIGGS_MOTION_LOADING;
        if (source.exhausted) out.flags |= HIGGS_MOTION_EXHAUSTED;
        for (std::size_t i = 0; i < source.hands.size(); ++i) {
            auto& hand = out.hands[i]; const auto& snapshot = source.hands[i];
            hand.physicalHand = i == 0 ? HIGGS_MOTION_LEFT : HIGGS_MOTION_RIGHT;
            hand.status = PublicStatus(snapshot.status);
            hand.acceptedObservation = snapshot.lastAcceptedObservation;
            hand.observedAtNs = snapshot.observedAtNs;
            if (hand.acceptedObservation) {
                hand.sourceFlags = HIGGS_MOTION_SOURCE_POSE_CALLBACK | HIGGS_MOTION_NO_HARDWARE_TIMESTAMP;
                if (source.copiedAtNs >= snapshot.observedAtNs) {
                    hand.validFields |= HIGGS_MOTION_AGE_KNOWN;
                    hand.ageNs = source.copiedAtNs - snapshot.observedAtNs;
                }
            }
            if (snapshot.available && snapshot.status == Status::Valid) {
                out.availableHands |= hand.physicalHand;
                hand.validFields |= HIGGS_MOTION_LINEAR_VALID | HIGGS_MOTION_ANGULAR_VALID |
                    HIGGS_MOTION_MEAN_LINEAR_VALID | HIGGS_MOTION_MEAN_SPEED_VALID | HIGGS_MOTION_PEAK_VALID;
                hand.latestLinear = Vector(snapshot.latestLinear); hand.latestAngular = Vector(snapshot.latestAngular);
                hand.meanLinear = Vector(snapshot.averageVelocity); hand.peakLinear = Vector(snapshot.peakVelocity);
                hand.meanSpeed = snapshot.averageSpeed;
                hand.historyObservationCount = snapshot.historyObservationCount;
            }
        }
        return out;
    }
}

namespace {
    HIGGS_MotionResult HIGGS_MOTION_CALL CopyVelocitySnapshot(HIGGS_MotionSnapshot* output) {
        if (!output) return HIGGS_MOTION_INVALID_ARGUMENT;
        if (output->structSize < sizeof(*output)) return HIGGS_MOTION_BUFFER_TOO_SMALL;
        try {
            const auto snapshot = HiggsMotion::VelocitySnapshot(HiggsMotion::Tracking().CopyObservation());
            std::memcpy(output, &snapshot, sizeof(snapshot));
            return HIGGS_MOTION_OK;
        }
        catch (...) { return HIGGS_MOTION_INTERNAL_ERROR; }
    }
}
extern "C" HIGGS_MotionResult HIGGS_MOTION_CALL HIGGS_QueryMotionInterface(
    uint32_t version, uint32_t tableSize, HIGGS_MotionInterface* output) {
    if (version != HIGGS_MOTION_ABI_VERSION) return HIGGS_MOTION_UNSUPPORTED_VERSION;
    if (!output) return HIGGS_MOTION_INVALID_ARGUMENT;
    if (tableSize < sizeof(*output)) return HIGGS_MOTION_BUFFER_TOO_SMALL;
    const HIGGS_MotionInterface table{sizeof(table), HIGGS_MOTION_ABI_VERSION,
        HIGGS_MOTION_CAP_VELOCITY | HIGGS_MOTION_CAP_PRODUCER_AGE |
        HIGGS_MOTION_CAP_PAIRED_CONTINUITY | HIGGS_MOTION_CAP_FIVE_SAMPLE_HISTORY,
        HIGGS_MOTION_HISTORY_SIZE, 0, CopyVelocitySnapshot};
    std::memcpy(output, &table, sizeof(table));
    return HIGGS_MOTION_OK;
}
