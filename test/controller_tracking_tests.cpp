#include "controller_tracking.h"
#include <atomic>
#include <barrier>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
    using namespace HiggsMotion;
    int checks = 0;
    void Check(bool value) { ++checks; if (!value) throw std::runtime_error("motion core check " + std::to_string(checks)); }
    bool Equal(Vector3 a, Vector3 b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
    Event World(std::uintptr_t world) { Event event; event.kind = EventKind::World; event.world = world; return event; }
    Event Sample(ObservationTicket ticket, float value = 5.f) {
        Event event; event.expectedGeneration = ticket.generation; event.observationSequence = ticket.sequence; event.observedAtNs = 100;
        event.hands = { Observation{Status::Valid, 1, {value, 0, 0}, {0, value, 0}},
            Observation{Status::Valid, 2, {-value, 0, 0}, {0, -value, 0}} };
        return event;
    }
    Event Issue(State& state, float value = 5.f) {
        const auto issued = state.lastIssuedObservation;
        state = Next(state, {EventKind::BeginObservation});
        return Sample(!state.exhausted && state.lastIssuedObservation > issued ?
            ObservationTicket{state.generation, state.lastIssuedObservation} : ObservationTicket{}, value);
    }
    void Transitions() {
        State state;
        Check(!Next(state, Issue(state)).hands[0].snapshot.available);
        state = Next(state, World(7));
        for (int i = 0; i < 5; ++i) state = Next(state, Issue(state));
        Check(state.hands[0].snapshot.averageSpeed == 5);
        Check(Equal(state.hands[0].snapshot.averageVelocity, {5,0,0}));
        Check(Equal(state.hands[0].snapshot.peakVelocity, {5,0,0}));
        Check(Equal(state.hands[0].snapshot.latestAngular, {0,5,0}));
        auto bad = Issue(state, 8); bad.hands[0].status = Status::InvalidPose;
        state = Next(state, bad);
        Check(!state.hands[0].snapshot.available && state.hands[1].snapshot.available);
        Check(Equal(state.hands[0].snapshot.peakVelocity, {}));
        for (auto value : state.hands[0].linear) Check(Equal(value, {}));
        for (auto value : state.hands[0].angular) Check(Equal(value, {}));
        Check(state.hands[1].snapshot.latestLinear.x == -8);
        state = Next(state, Issue(state, 2));
        Check(state.hands[0].snapshot.averageSpeed == 0.4f); // strict restart after one invalid observation
        auto changed = Issue(state, 3); changed.hands[1].device = 4;
        state = Next(state, changed);
        Check(state.hands[1].snapshot.averageSpeed == 0.6f);
        Check(state.hands[1].snapshot.peakVelocity.x == -3);
        auto old = Issue(state);
        state = Next(state, World(9));
        state = Next(state, old);
        Check(!state.hands[0].snapshot.available && state.world == 9);
        state = Next(state, Issue(state));
        state = Next(state, {EventKind::BeginLoad});
        Check(state.loading && !state.world && !state.hands[0].snapshot.available);
        state = Next(state, World(9)); Check(!state.world);
        state = Next(state, {EventKind::EndLoad}); Check(!state.loading && !state.world);
        state = Next(state, World(9)); state = Next(state, Issue(state));
        state = Next(state, {EventKind::NewGame}); Check(!state.world && !state.hands[1].snapshot.available);
        state.generation = (std::numeric_limits<std::uint64_t>::max)();
        state = Next(state, World(9)); Check(state.exhausted && !state.world);
        state = Next(state, Issue(state)); Check(!state.hands[0].snapshot.available);
    }
    void ConversionAndArithmetic() {
        Rotation identity{{{1,0,0},{0,1,0},{0,0,1}}};
        auto converted = Convert(1, {1,2,3}, {4,5,6}, identity);
        Check(converted.status == Status::Valid);
        Check(Equal(converted.linear, {1,-3,2}) && Equal(converted.angular, {4,-6,5}));
        Rotation turn{{{0,-1,0},{1,0,0},{0,0,1}}};
        Check(Equal(Convert(1,{1,2,3},{},turn).linear,{3,1,2}));
        for (float bad : {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
            Check(Convert(1,{bad,0,0},{},identity).status == Status::NonFinite);
            Check(Convert(1,{},{0,bad,0},identity).status == Status::NonFinite);
            auto invalid = identity; invalid[1][0] = bad;
            Check(Convert(1,{},{},invalid).status == Status::InvalidConversion);
        }
        auto invalid = identity; invalid[2] = {}; Check(!IsRotation(invalid));
        invalid = identity; invalid[2][2] = -1; Check(!IsRotation(invalid));
        State state = Next({}, World(3));
        auto overflow = Issue(state, 1e30f);
        state = Next(state, overflow); Check(!state.hands[0].snapshot.available);
        auto noDevice = Issue(state); noDevice.hands[0].device = kNoDevice;
        state = Next(state, noDevice); Check(!state.hands[0].snapshot.available && state.hands[1].snapshot.available);
        Check(NowNs() <= NowNs());
    }
    void ObservationOrdering() {
        ControllerTracking owner; owner.Apply(World(1));
        const auto older = owner.BeginObservation(), newer = owner.BeginObservation();
        Check(older.generation == newer.generation && newer.sequence > older.sequence);
        auto invalid = Sample(newer, 8); invalid.hands[0].status = Status::InvalidPose;
        owner.Apply(invalid); owner.Apply(Sample(older, 3));
        auto sample = owner.Copy(true);
        Check(!sample.available && sample.lastAcceptedObservation == newer.sequence && sample.observedAtNs == 100);
        Check(owner.Copy(false).available && owner.Copy(false).latestLinear.x == -8);
        const auto olderInvalid = owner.BeginObservation(), newerValid = owner.BeginObservation();
        owner.Apply(Sample(newerValid, 9));
        auto stale = Sample(olderInvalid); stale.hands[0].status = Status::InvalidPose;
        owner.Apply(stale);
        sample = owner.Copy(true);
        Check(sample.available && sample.latestLinear.x == 9 && sample.lastAcceptedObservation == newerValid.sequence);
        // Every sample above has the SAME timestamp. Order is the ticket, never age.
        auto first = Sample(owner.BeginObservation(), 2); owner.Apply(first);
        auto second = Sample(owner.BeginObservation(), 4); owner.Apply(second);
        Check(owner.Copy(true).latestLinear.x == 4 && owner.Copy(true).observedAtNs == first.observedAtNs);
        second.hands[0].status = Status::InvalidPose; owner.Apply(second);
        Check(owner.Copy(true).available); // A duplicate accepted ticket cannot republish changed data.
        second.observationSequence += 1; owner.Apply(second);
        Check(owner.Copy(true).available); // An unissued future ticket is not a publication authority.
        const auto preReset = owner.BeginObservation();
        owner.Apply({EventKind::BeginLoad}); owner.Apply({EventKind::EndLoad}); owner.Apply(World(1));
        const auto postReset = owner.BeginObservation();
        Check(postReset.generation != preReset.generation && postReset.sequence > preReset.sequence);
        auto latest = Sample(postReset); latest.hands[0].status = Status::Disconnected;
        owner.Apply(latest); owner.Apply(Sample(preReset));
        Check(!owner.Copy(true).available && owner.Copy(true).lastAcceptedObservation == postReset.sequence);

        const auto maximum = (std::numeric_limits<std::uint64_t>::max)();
        State state = Next({}, World(1)); state.lastIssuedObservation = maximum - 1;
        state = Next(state, {EventKind::BeginObservation});
        Check(state.lastIssuedObservation == maximum && !state.exhausted);
        const auto finalTicket = ObservationTicket{state.generation, maximum};
        state = Next(state, Sample(finalTicket));
        Check(state.hands[0].snapshot.available && state.lastAcceptedObservation == maximum);
        state = Next(state, {EventKind::BeginObservation});
        Check(state.exhausted && !state.world && state.lastIssuedObservation == maximum && !state.hands[0].snapshot.available);
        state = Next(state, Sample(finalTicket));
        Check(!state.hands[0].snapshot.available && state.lastIssuedObservation == maximum);
        state = Next(state, {EventKind::NewGame}); state = Next(state, World(2));
        state = Next(state, {EventKind::BeginObservation});
        Check(state.exhausted && !state.world && state.lastIssuedObservation == maximum);
        state = Next({}, World(1)); state.lastIssuedObservation = maximum;
        state = Next(state, {EventKind::BeginObservation});
        auto outstanding = Sample({state.generation, maximum}); state = Next(state, outstanding);
        Check(state.exhausted && !state.hands[0].snapshot.available); // Exhaustion rejects even an outstanding final ticket.
    }
    void ConcurrentCopies() {
        ControllerTracking owner; owner.Apply(World(1));
        const auto oldTicket = owner.BeginObservation();
        std::atomic<bool> done = false, okay = true;
        std::barrier start(3);
        std::atomic<unsigned> phase = 0;
        std::array<std::atomic<unsigned>, 2> observed{}, reads{};
        std::thread writer([&] {
            start.arrive_and_wait();
            for (unsigned round = 1; round <= 64; ++round) {
                for (unsigned i = 0; i < 128; ++i)
                    owner.Apply(Sample(owner.BeginObservation(), static_cast<float>(i % 31 + 1)));
                phase = round;
                // Every reader must copy a publication in every phase before
                // this producer may continue. Scheduling cannot skip the test.
                while (observed[0] < round || observed[1] < round) std::this_thread::yield();
            }
            done = true;
        });
        const auto reader = [&](unsigned index) {
            start.arrive_and_wait();
            while (!done) {
                const auto round = phase.load();
                const auto pair = owner.CopyAll();
                if (pair[0].available && (pair[0].latestLinear.x != pair[0].latestAngular.y ||
                    pair[0].latestLinear.x != -pair[1].latestLinear.x || pair[1].latestLinear.x != pair[1].latestAngular.y)) okay = false;
                if (pair[0].available) ++reads[index];
                observed[index] = round;
            }
        };
        std::thread a(reader, 0), b(reader, 1); writer.join(); a.join(); b.join(); Check(okay);
        Check(observed[0] == 64 && observed[1] == 64 && reads[0] >= 64 && reads[1] >= 64);
        Check(owner.Copy(true).available && owner.Copy(true).lastAcceptedObservation > oldTicket.sequence);
        auto copy = owner.Copy(true); copy.latestLinear.x = 999;
        Check(owner.Copy(true).latestLinear.x != 999);
        owner.Apply({EventKind::NewGame}); owner.Apply(Sample(oldTicket));
        Check(!owner.Copy(true).available && !owner.Copy(false).available);
        owner.Apply(World(1)); done = false;
        std::atomic<bool> resetDone = false;
        std::atomic<unsigned> mixedPhase = 0, resetPhase = 0, mixedObserved = 0, resetObserved = 0, mixedReads = 0;
        std::barrier mixedStart(3);
        std::thread mixedWriter([&] {
            mixedStart.arrive_and_wait();
            for (unsigned round = 1; round <= 64; ++round) {
                for (unsigned i = 0; i < 80; ++i) {
                    auto event = Sample(owner.BeginObservation());
                    if (i % 2) event.hands[0].status = Status::InvalidPose;
                    owner.Apply(event);
                }
                mixedPhase = round;
                while (mixedObserved < round) std::this_thread::yield();
            }
            done = true;
        });
        std::thread resetter([&] {
            mixedStart.arrive_and_wait();
            for (unsigned round = 1; round <= 64; ++round) {
                for (unsigned i = 0; i < 16; ++i) {
                    owner.Apply({EventKind::BeginLoad}); owner.Apply({EventKind::EndLoad}); owner.Apply(World(1));
                }
                resetPhase = round;
                while (resetObserved < round) std::this_thread::yield();
            }
            resetDone = true;
        });
        std::thread mixedReader([&] {
            mixedStart.arrive_and_wait();
            while (!done || !resetDone) {
                const auto writeRound = mixedPhase.load(), resetRound = resetPhase.load();
                for (const auto& sample : owner.CopyAll()) {
                    if (!sample.available && (!Equal(sample.latestLinear, {}) || !Equal(sample.latestAngular, {}) || sample.averageSpeed != 0)) okay = false;
                    if (sample.available && std::abs(sample.latestLinear.x) != std::abs(sample.latestAngular.y)) okay = false;
                }
                ++mixedReads; mixedObserved = writeRound; resetObserved = resetRound;
            }
        });
        mixedWriter.join(); resetter.join(); mixedReader.join(); Check(okay);
        Check(mixedObserved == 64 && resetObserved == 64 && mixedReads >= 64);
    }
}
int main() {
    try { Transitions(); ConversionAndArithmetic(); ObservationOrdering(); ConcurrentCopies(); std::cout << "PASS motion core " << checks << " checks\n"; return 0; }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
