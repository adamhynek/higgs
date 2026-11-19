#include "controller_tracking.h"

#include <cmath>
#include <chrono>
#include <limits>
#include <utility>

namespace HiggsMotion {
    namespace {
        bool Finite(const Vector3& value) { return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z); }
        float Length(const Vector3& value) { return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z); }
        Vector3 Add(const Vector3& a, const Vector3& b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
        Vector3 Divide(const Vector3& value, float scale) { return { value.x / scale, value.y / scale, value.z / scale }; }
        Vector3 Multiply(const Rotation& rotation, const Vector3& value) {
            return { rotation[0][0] * value.x + rotation[0][1] * value.y + rotation[0][2] * value.z,
                rotation[1][0] * value.x + rotation[1][1] * value.y + rotation[1][2] * value.z,
                rotation[2][0] * value.x + rotation[2][1] * value.y + rotation[2][2] * value.z };
        }
        bool ValidRotation(const Rotation& rotation) {
            // Both inputs to the native conversion are rotation matrices. This
            // numerical tolerance is not a tracking-loss time/grace policy.
            constexpr float tolerance = 0.001f;
            for (const auto& row : rotation) for (float value : row) if (!std::isfinite(value)) return false;
            for (std::size_t i = 0; i < 3; ++i) for (std::size_t j = 0; j < 3; ++j) {
                float product = 0;
                for (std::size_t k = 0; k < 3; ++k) product += rotation[i][k] * rotation[j][k];
                if (std::abs(product - (i == j ? 1.f : 0.f)) > tolerance) return false;
            }
            const auto& a = rotation[0]; const auto& b = rotation[1]; const auto& c = rotation[2];
            const float determinant = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
            return std::abs(determinant - 1.f) <= tolerance;
        }
        History Invalid(Status status, std::uint32_t device, std::uint64_t now) {
            History result;
            result.snapshot.status = status; result.snapshot.device = device; result.snapshot.observedAtNs = now;
            return result;
        }
        History Observe(const History& previous, const Observation& observation, std::uint64_t now) {
            if (observation.status != Status::Valid) return Invalid(observation.status, observation.device, now);
            if (observation.device == kNoDevice || !Finite(observation.linear) || !Finite(observation.angular) ||
                !std::isfinite(Length(observation.linear)) || !std::isfinite(Length(observation.angular)))
                return Invalid(Status::NonFinite, observation.device, now);
            History next = previous.snapshot.available && previous.snapshot.device == observation.device ? previous : History{};
            for (std::size_t i = kHistorySize - 1; i > 0; --i) { next.linear[i] = next.linear[i - 1]; next.angular[i] = next.angular[i - 1]; }
            next.linear[0] = observation.linear; next.angular[0] = observation.angular;
            const auto previousCount = next.snapshot.historyObservationCount;
            auto& snapshot = next.snapshot;
            snapshot = {}; snapshot.available = true; snapshot.status = Status::Valid; snapshot.device = observation.device;
            snapshot.latestLinear = observation.linear; snapshot.latestAngular = observation.angular; snapshot.observedAtNs = now;
            snapshot.historyObservationCount = previousCount < kHistorySize ? previousCount + 1 : static_cast<std::uint32_t>(kHistorySize);
            float largestSpeed = -1;
            std::size_t largestIndex = 0;
            for (std::size_t i = 0; i < kHistorySize; ++i) {
                snapshot.averageVelocity = Add(snapshot.averageVelocity, next.linear[i]);
                const float speed = Length(next.linear[i]); snapshot.averageSpeed += speed;
                if (speed > largestSpeed) { largestSpeed = speed; largestIndex = i; }
            }
            snapshot.averageVelocity = Divide(snapshot.averageVelocity, static_cast<float>(kHistorySize));
            snapshot.averageSpeed /= static_cast<float>(kHistorySize);
            snapshot.peakVelocity = largestIndex == 0 || largestIndex == kHistorySize - 1 ? next.linear[largestIndex] :
                Divide(Add(Add(next.linear[largestIndex - 1], next.linear[largestIndex]), next.linear[largestIndex + 1]), 3.f);
            if (!Finite(snapshot.averageVelocity) || !Finite(snapshot.peakVelocity) || !std::isfinite(snapshot.averageSpeed))
                return Invalid(Status::NonFinite, observation.device, now);
            return next;
        }
        void Reset(State& candidate) {
            candidate.hands = {};
            for (auto& hand : candidate.hands) hand.snapshot.status = Status::WorldUnavailable;
            candidate.world = 0;
            if (candidate.generation == (std::numeric_limits<std::uint64_t>::max)()) candidate.exhausted = true;
            else ++candidate.generation;
        }
    }
    bool IsRotation(const Rotation& rotation) { return ValidRotation(rotation); }
    Observation Convert(std::uint32_t device, const Vector3& linear, const Vector3& angular, const Rotation& worldRotation) {
        if (!ValidRotation(worldRotation)) return { Status::InvalidConversion, device, {}, {} };
        // OpenVR x/y/z -> Skyrim x/-z/y. Velocities remain m/s and rad/s.
        Observation result{ Status::Valid, device, Multiply(worldRotation, { linear.x, -linear.z, linear.y }),
            Multiply(worldRotation, { angular.x, -angular.z, angular.y }) };
        if (!Finite(linear) || !Finite(angular) || !Finite(result.linear) || !Finite(result.angular)) result.status = Status::NonFinite;
        return result;
    }
    State Next(const State& previous, const Event& event) {
        State candidate = previous;
        if (event.kind == EventKind::Observe) {
            if (!candidate.world || candidate.loading || candidate.exhausted || event.expectedGeneration != candidate.generation ||
                !event.observationSequence || event.observationSequence > candidate.lastIssuedObservation ||
                event.observationSequence <= candidate.lastAcceptedObservation) return candidate;
            candidate.lastAcceptedObservation = event.observationSequence;
            for (std::size_t i = 0; i < candidate.hands.size(); ++i) {
                candidate.hands[i] = Observe(previous.hands[i], event.hands[i], event.observedAtNs);
                candidate.hands[i].snapshot.lastAcceptedObservation = event.observationSequence;
            }
        }
        else if (event.kind == EventKind::BeginObservation) {
            if (!candidate.world || candidate.loading || candidate.exhausted) return candidate;
            if (candidate.lastIssuedObservation == (std::numeric_limits<std::uint64_t>::max)()) {
                Reset(candidate); candidate.exhausted = true; // Never wrap/reuse a ticket or accept an outstanding one.
            }
            else ++candidate.lastIssuedObservation;
        }
        else if (event.kind == EventKind::World) {
            if (!candidate.loading && candidate.world != event.world) { Reset(candidate); if (!candidate.exhausted) candidate.world = event.world; }
        }
        else if (event.kind == EventKind::BeginLoad) { Reset(candidate); candidate.loading = true; }
        else if (event.kind == EventKind::EndLoad) { candidate.loading = false; }
        else if (event.kind == EventKind::NewGame) { Reset(candidate); candidate.loading = false; }
        return candidate;
    }
    void ControllerTracking::Commit(State candidate) { state_ = std::move(candidate); }
    void ControllerTracking::Apply(const Event& event) { std::scoped_lock lock(mutex_); Commit(Next(state_, event)); }
    ObservationTicket ControllerTracking::BeginObservation() {
        std::scoped_lock lock(mutex_);
        auto candidate = Next(state_, {EventKind::BeginObservation});
        const ObservationTicket ticket = !candidate.exhausted && candidate.lastIssuedObservation > state_.lastIssuedObservation ?
            ObservationTicket{candidate.generation, candidate.lastIssuedObservation} : ObservationTicket{};
        Commit(std::move(candidate));
        return ticket;
    }
    Snapshot ControllerTracking::Copy(bool isLeft) const { std::scoped_lock lock(mutex_); return state_.hands[isLeft ? 0 : 1].snapshot; }
    std::array<Snapshot, 2> ControllerTracking::CopyAll() const { std::scoped_lock lock(mutex_); return { state_.hands[0].snapshot, state_.hands[1].snapshot }; }
    CopiedTracking ControllerTracking::CopyObservation() const {
        std::scoped_lock lock(mutex_);
        return { {state_.hands[0].snapshot, state_.hands[1].snapshot}, state_.generation,
            state_.lastAcceptedObservation, NowNs(), state_.world != 0, state_.loading, state_.exhausted };
    }
    ControllerTracking& Tracking() { static ControllerTracking tracking; return tracking; }
    std::uint64_t NowNs() {
        return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
    }
}
