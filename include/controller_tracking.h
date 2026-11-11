#pragma once

// Internal numeric tracking history, independent of SKSE/Havok and public mod ABIs.
#include <array>
#include <cstdint>
#include <mutex>

namespace HiggsMotion {
    constexpr std::size_t kHistorySize = 5;
    constexpr std::uint32_t kNoDevice = UINT32_MAX;
    struct Vector3 { float x = 0, y = 0, z = 0; };
    using Rotation = std::array<std::array<float, 3>, 3>;
    enum class Status { Unavailable, Valid, InvalidPose, Disconnected, InvalidConversion, NonFinite, WorldUnavailable };
    struct Observation {
        Status status = Status::Unavailable;
        std::uint32_t device = kNoDevice;
        Vector3 linear, angular;
    };
    struct Snapshot {
        bool available = false;
        Status status = Status::Unavailable;
        std::uint32_t device = kNoDevice;
        Vector3 latestLinear, latestAngular, averageVelocity, peakVelocity;
        float averageSpeed = 0;
        std::uint32_t historyObservationCount = 0; // Accepted consecutive valid observations, saturated at five.
        std::uint64_t observedAtNs = 0;
        std::uint64_t lastAcceptedObservation = 0; // Internal order, independent of observation time.
    };
    struct History {
        std::array<Vector3, kHistorySize> linear{}, angular{};
        Snapshot snapshot;
    };
    struct State {
        // Physical left = 0, physical right = 1. A world identity is only a
        // continuity observation from Update, never dereferenced by this owner.
        std::array<History, 2> hands{};
        std::uintptr_t world = 0;
        std::uint64_t generation = 1;
        std::uint64_t lastIssuedObservation = 0, lastAcceptedObservation = 0;
        bool loading = false, exhausted = false;
    };
    struct ObservationTicket { std::uint64_t generation = 0, sequence = 0; };
    struct CopiedTracking {
        std::array<Snapshot, 2> hands{};
        std::uint64_t generation = 0, lastAcceptedObservation = 0, copiedAtNs = 0;
        bool worldObserved = false, loading = false, exhausted = false;
    };
    enum class EventKind { Observe, World, BeginLoad, EndLoad, NewGame, BeginObservation };
    struct Event {
        EventKind kind = EventKind::Observe;
        std::array<Observation, 2> hands{};
        std::uint64_t expectedGeneration = 0, observedAtNs = 0, observationSequence = 0;
        std::uintptr_t world = 0;
    };

    // Pure conversion/transition. Invalid input never becomes stationary-valid.
    Observation Convert(std::uint32_t device, const Vector3& linear, const Vector3& angular, const Rotation& worldRotation);
    bool IsRotation(const Rotation& rotation);
    State Next(const State& previous, const Event& event);

    class ControllerTracking {
    public:
        ObservationTicket BeginObservation(); // Captures scope and reserves order under one lock, before engine work.
        Snapshot Copy(bool isLeft) const;
        std::array<Snapshot, 2> CopyAll() const;
        CopiedTracking CopyObservation() const; // Pair, lifecycle and producer time under the same lock.
        void Apply(const Event& event);
    private:
        void Commit(State candidate);
        mutable std::mutex mutex_;
        State state_;
    };
    ControllerTracking& Tracking();
    std::uint64_t NowNs();
}
