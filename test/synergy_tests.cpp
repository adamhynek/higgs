#include "synergy_core.h"
#include "synergy_policy.h"
#include "synergy_runtime.h"

#include <atomic>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <vector>

extern "C" int synergy_c_consumer(HIGGS_QuerySynergyInterfaceFn);

static_assert(std::is_trivially_copyable_v<HIGGS_SynergyInterface>);
static_assert(std::is_standard_layout_v<HIGGS_SynergyEffectiveState>);
static_assert(sizeof(HIGGS_SynergyEffectiveState) == 104);

namespace {
    using HiggsSynergy::RestrictionService;
    void Check(bool condition, const char* message) {
        if (!condition) throw std::runtime_error(message);
    }
    HIGGS_SynergyClient Client(RestrictionService &service, const char* name = "client") {
        HIGGS_SynergyClientInfo info{ sizeof(info), 0, {} };
        const auto length = (std::min)(std::strlen(name), sizeof(info.name) - 1);
        std::memcpy(info.name, name, length);
        HIGGS_SynergyClient client = 0;
        Check(service.RegisterClient(info, 1000, client) == HIGGS_SYNERGY_OK, "register");
        return client;
    }
    HIGGS_SynergyLeaseRequest Request(RestrictionService &service, std::uint32_t hands, std::uint32_t flags,
        std::uint32_t duration = 60000) {
        return { sizeof(HIGGS_SynergyLeaseRequest), hands, flags, duration, service.ReadEffectiveState(1000).worldGeneration };
    }
    HIGGS_SynergyLeaseInfo Acquire(RestrictionService &service, HIGGS_SynergyClient client,
        std::uint32_t hands, std::uint32_t flags, std::uint32_t duration = 60000) {
        auto request = Request(service, hands, flags, duration);
        HIGGS_SynergyLeaseInfo lease{};
        Check(service.AcquireLease(client, request, 1000, lease) == HIGGS_SYNERGY_OK, "acquire");
        Check(lease.lease != 0, "nonzero lease");
        return lease;
    }

    void OverlapLegacyAndPossession() {
        RestrictionService service;
        service.ObserveWorld(1, 1000);
        auto a = Client(service, "climbing"), b = Client(service, "harvesting");
        auto one = Acquire(service, a, HIGGS_SYNERGY_HAND_LEFT, HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS);
        auto left = service.ReadEffectiveState(1000).hands[0];
        Check(!HiggsSynergy::MayStartHandInteraction(left), "new admission suppressed");
        Check(HiggsSynergy::MayContinueHeldInteraction(left), "scoped lease preserves held object");
        Check(!service.ReadEffectiveState(1000).hands[1].handInteractionsDisabled, "physical right independent");
        service.SetLegacyHandDisabled(true, true);
        Check(!HiggsSynergy::MayContinueHeldInteraction(service.ReadEffectiveState(1000).hands[0]), "legacy forced drop retained");
        auto both = Acquire(service, b, HIGGS_SYNERGY_HAND_BOTH, HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS);
        service.SetLegacyHandDisabled(true, false);
        auto state = service.ReadEffectiveState(1000);
        Check(state.hands[0].handInteractionsDisabled && state.hands[1].handInteractionsDisabled, "legacy enable cannot clear scoped lease");
        Check(HiggsSynergy::MayContinueHeldInteraction(state.hands[0]), "legacy enable restores continuation only");
        Check(service.ReleaseLease(a, both.lease, 1000) == HIGGS_SYNERGY_WRONG_OWNER, "foreign release rejected");
        Check(service.ReleaseLease(a, one.lease, 1000) == HIGGS_SYNERGY_OK, "release own");
        Check(service.ReadEffectiveState(1000).hands[0].handInteractionsDisabled, "other owner's overlap remains");
        Check(service.ReleaseLease(a, one.lease, 1000) == HIGGS_SYNERGY_INVALID_LEASE, "duplicate release rejected");
        Check(service.ReleaseLease(b, both.lease, 1000) == HIGGS_SYNERGY_OK, "release both atomically");
        state = service.ReadEffectiveState(1000);
        Check(!state.hands[0].handInteractionsDisabled && !state.hands[1].handInteractionsDisabled, "no claim after last release");
        service.SetLegacyHandDisabled(true, true);
        one = Acquire(service, a, HIGGS_SYNERGY_HAND_LEFT, HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS);
        Check(service.ReleaseLease(a, one.lease, 1000) == HIGGS_SYNERGY_OK, "release scoped under legacy");
        Check(service.ReadEffectiveState(1000).hands[0].handInteractionsDisabled, "release cannot clear legacy");
    }

    void CollisionPrecedenceAndRemovableForce() {
        RestrictionService service;
        service.ObserveWorld(1, 1000);
        auto a = Client(service), b = Client(service);
        auto force = Acquire(service, a, HIGGS_SYNERGY_HAND_RIGHT, HIGGS_SYNERGY_FORCE_WEAPON_COLLISION);
        auto right = service.ReadEffectiveState(1000).hands[1];
        Check(!HiggsSynergy::ShouldDisableWeaponCollision(right, true, true), "scoped force bypasses hit window");
        Check(HiggsSynergy::ShouldDisableWeaponCollision(right, false, false), "sheathing overrides force");
        auto disable = Acquire(service, b, HIGGS_SYNERGY_HAND_RIGHT, HIGGS_SYNERGY_SUPPRESS_WEAPON_COLLISION);
        right = service.ReadEffectiveState(1000).hands[1];
        Check(right.weaponCollisionDisabled && !right.forceWeaponCollisionEnabled, "restriction wins over force");
        Check(HiggsSynergy::ShouldDisableWeaponCollision(right, true, false), "engine filter uses restriction");
        Check(service.ReleaseLease(b, disable.lease, 1000) == HIGGS_SYNERGY_OK, "remove suppression");
        Check(service.ReadEffectiveState(1000).hands[1].forceWeaponCollisionEnabled, "other force restored");
        Check(service.ReleaseLease(a, force.lease, 1000) == HIGGS_SYNERGY_OK, "remove scoped force");
        Check(HiggsSynergy::ShouldDisableWeaponCollision(service.ReadEffectiveState(1000).hands[1], true, true), "normal hit window restored");
        service.ForceLegacyWeaponCollision(false);
        service.SetLegacyWeaponCollisionDisabled(false, true);
        Check(service.ReadEffectiveState(1000).hands[1].weaponCollisionDisabled, "legacy disable wins over legacy force");
        service.SetLegacyWeaponCollisionDisabled(false, false);
        disable = Acquire(service, b, HIGGS_SYNERGY_HAND_RIGHT, HIGGS_SYNERGY_SUPPRESS_WEAPON_COLLISION);
        service.SetLegacyWeaponCollisionDisabled(false, false);
        Check(service.ReadEffectiveState(1000).hands[1].weaponCollisionDisabled, "legacy collision enable cannot clear lease");
        Check(service.ReleaseLease(b, disable.lease, 1000) == HIGGS_SYNERGY_OK, "remove lease under legacy force");
        right = service.ReadEffectiveState(1000).hands[1];
        Check(right.legacyForceWeaponCollisionEnabled && right.forceWeaponCollisionEnabled, "legacy permanent latch retained");
    }

    void RenewalExpiryAndClockOrdering() {
        RestrictionService service;
        service.ObserveWorld(1, 1000);
        auto a = Client(service), b = Client(service);
        auto lease = Acquire(service, a, HIGGS_SYNERGY_HAND_BOTH, HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS, 100);
        HIGGS_SynergyRenewRequest renewal{ sizeof(renewal), 100, lease.worldGeneration, lease.lease };
        HIGGS_SynergyLeaseInfo renewed{};
        Check(service.RenewLease(b, renewal, 1080, renewed) == HIGGS_SYNERGY_WRONG_OWNER, "foreign renewal rejected");
        Check(service.RenewLease(a, renewal, 1080, renewed) == HIGGS_SYNERGY_OK, "renew live lease");
        Check(renewed.lease == lease.lease && renewed.expiresAtMs == 1180, "renew preserves token footprint");
        auto state = service.ReadEffectiveState(1100);
        Check(state.hands[0].handInteractionsDisabled && state.hands[1].handInteractionsDisabled, "no authority gap during renewal");
        auto generation = state.stateGeneration;
        Check(service.ReadEffectiveState(1090).sampledAtMs == 1100, "out-of-order clock samples clamped");
        state = service.ReadEffectiveState(1180);
        Check(!state.hands[0].handInteractionsDisabled && !state.hands[1].handInteractionsDisabled, "expiry at exact deadline clears both");
        Check(state.stateGeneration > generation, "expiry advances generation");
        Check(service.RenewLease(a, renewal, 1180, renewed) == HIGGS_SYNERGY_INVALID_LEASE, "expired token cannot renew");
        auto replacement = Acquire(service, a, HIGGS_SYNERGY_HAND_LEFT, HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS);
        Check(replacement.lease != lease.lease, "tokens never reused after expiry");
        Check(service.ReleaseLease(a, lease.lease, 1180) == HIGGS_SYNERGY_INVALID_LEASE, "stale token cannot release replacement");
    }

    void CapacityAndAllOrNone() {
        RestrictionService service;
        service.ObserveWorld(1, 1000);
        auto a = Client(service);
        std::vector<HIGGS_SynergyLeaseInfo> leases;
        for (unsigned i = 0; i < HIGGS_SYNERGY_MAX_LEASES; ++i)
            leases.push_back(Acquire(service, a, HIGGS_SYNERGY_HAND_RIGHT, HIGGS_SYNERGY_FORCE_WEAPON_COLLISION));
        auto request = Request(service, HIGGS_SYNERGY_HAND_BOTH, HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS);
        HIGGS_SynergyLeaseInfo refused{};
        auto before = service.ReadEffectiveState(1000);
        Check(service.AcquireLease(a, request, 1000, refused) == HIGGS_SYNERGY_CAPACITY && refused.lease == 0, "capacity rejects whole two-hand request");
        auto after = service.ReadEffectiveState(1000);
        Check(after.stateGeneration == before.stateGeneration && !after.hands[0].handInteractionsDisabled &&
            !after.hands[1].handInteractionsDisabled, "no partial state on failed acquisition");
        Check(service.ReleaseLease(a, leases.back().lease, 1000) == HIGGS_SYNERGY_OK, "free one slot");
        auto both = Acquire(service, a, HIGGS_SYNERGY_HAND_BOTH, HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS);
        after = service.ReadEffectiveState(1000);
        Check(after.hands[0].activeScopedLeases == 1 && after.hands[1].activeScopedLeases == HIGGS_SYNERGY_MAX_LEASES &&
            after.hands[0].handInteractionsDisabled && after.hands[1].handInteractionsDisabled, "one slot covers both atomically");
        Check(service.UnregisterClient(a, 1000) == HIGGS_SYNERGY_OK, "client revoke clears all leases");
        after = service.ReadEffectiveState(1000);
        Check(after.hands[0].activeScopedLeases == 0 && after.hands[1].activeScopedLeases == 0, "all footprints revoked");
        Check(service.ReleaseLease(a, both.lease, 1000) == HIGGS_SYNERGY_INVALID_CLIENT, "unregistered owner stale");
        auto replacement = Client(service);
        Check(replacement != a, "client handles never reused");
        for (unsigned i = 1; i < HIGGS_SYNERGY_MAX_CLIENTS; ++i) Client(service);
        HIGGS_SynergyClientInfo info{ sizeof(info), 0, "over capacity" };
        HIGGS_SynergyClient extra = 999;
        Check(service.RegisterClient(info, 1000, extra) == HIGGS_SYNERGY_CAPACITY && extra == 0, "client bound enforced");
    }

    void ValidationIsNonMutating() {
        RestrictionService service;
        service.ObserveWorld(1, 1000);
        auto a = Client(service);
        auto request = Request(service, HIGGS_SYNERGY_HAND_LEFT, HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS);
        auto generation = service.ReadEffectiveState(1000).stateGeneration;
        HIGGS_SynergyLeaseInfo output{};
        for (auto hands : {0u, 4u, 0xffffffffu}) {
            request.physicalHands = hands;
            Check(service.AcquireLease(a, request, 1000, output) == HIGGS_SYNERGY_INVALID_ARGUMENT, "invalid physical hand mask");
        }
        request.physicalHands = HIGGS_SYNERGY_HAND_LEFT;
        for (auto flags : {0u, 8u, 0xffffffffu}) {
            request.flags = flags;
            Check(service.AcquireLease(a, request, 1000, output) == HIGGS_SYNERGY_INVALID_ARGUMENT, "unknown flag rejected");
        }
        request.flags = HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS;
        for (auto duration : {0u, 60001u, 0xffffffffu}) {
            request.durationMs = duration;
            Check(service.AcquireLease(a, request, 1000, output) == HIGGS_SYNERGY_INVALID_ARGUMENT, "duration bound");
        }
        Check(service.ReadEffectiveState(1000).stateGeneration == generation, "rejections leave state unchanged");
        request.durationMs = 60;
        Check(service.AcquireLease(a, request, (std::numeric_limits<std::uint64_t>::max)() - 20, output) ==
            HIGGS_SYNERGY_INVALID_ARGUMENT, "expiry addition checked");
    }

    void WorldAndGameLifecycle() {
        RestrictionService service;
        auto a = Client(service);
        auto request = Request(service, HIGGS_SYNERGY_HAND_BOTH, HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS);
        HIGGS_SynergyLeaseInfo output{};
        Check(service.AcquireLease(a, request, 1000, output) == HIGGS_SYNERGY_NOT_READY, "not ready before world");
        service.ObserveWorld(1, 1000);
        service.SetLegacyHandDisabled(false, true);
        service.ForceLegacyWeaponCollision(true);
        auto lease = Acquire(service, a, HIGGS_SYNERGY_HAND_BOTH, HIGGS_SYNERGY_SUPPRESS_WEAPON_COLLISION);
        service.BeginGameLoad();
        auto state = service.ReadEffectiveState(1000);
        Check(!state.worldReady && state.worldGeneration != lease.worldGeneration &&
            state.hands[0].activeScopedLeases == 0, "load immediately revokes scoped leases");
        service.ObserveWorld(1, 1000);
        Check(!service.ReadEffectiveState(1000).worldReady, "old world cannot reopen service while loading");
        service.EndGameLoad();
        service.ObserveWorld(1, 1000); // Same pointer can be reused after loading.
        state = service.ReadEffectiveState(1000);
        Check(state.worldReady && state.hands[1].legacyHandDisabled &&
            state.hands[0].legacyForceWeaponCollisionEnabled, "legacy state and registrations survive lifecycle");
        request.expectedWorldGeneration = lease.worldGeneration;
        Check(service.AcquireLease(a, request, 1000, output) == HIGGS_SYNERGY_STALE_WORLD, "old-world delayed acquisition rejected");
        Check(service.ReleaseLease(a, lease.lease, 1000) == HIGGS_SYNERGY_INVALID_LEASE, "revoked token cannot affect new world");
        lease = Acquire(service, a, HIGGS_SYNERGY_HAND_BOTH, HIGGS_SYNERGY_SUPPRESS_WEAPON_COLLISION);
        service.ObserveWorld(2, 1000);
        Check(service.ReadEffectiveState(1000).hands[0].activeScopedLeases == 0, "world replacement revokes");
        lease = Acquire(service, a, HIGGS_SYNERGY_HAND_LEFT, HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS);
        service.ObserveWorld(0, 1000);
        Check(!service.ReadEffectiveState(1000).worldReady && !service.ReadEffectiveState(1000).hands[0].activeScopedLeases, "world loss revokes");
        service.ObserveWorld(3, 1000);
        lease = Acquire(service, a, HIGGS_SYNERGY_HAND_LEFT, HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS);
        service.NewGame();
        Check(!service.ReadEffectiveState(1000).worldReady && !service.ReadEffectiveState(1000).hands[0].activeScopedLeases, "new game revokes");
        Check(service.CopyEffectiveState(a, 1000, state) == HIGGS_SYNERGY_OK, "registration survives new game");
    }

    void ConcurrentOwnersAndSnapshots() {
        RestrictionService service;
        service.ObserveWorld(1, 1000);
        auto baselineOwner = Client(service);
        Acquire(service, baselineOwner, HIGGS_SYNERGY_HAND_BOTH, HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS);
        std::array<HIGGS_SynergyClient, 6> clients{};
        for (auto &client : clients) client = Client(service);
        std::atomic<unsigned> failures{0};
        std::vector<std::thread> threads;
        auto request = Request(service, HIGGS_SYNERGY_HAND_BOTH, HIGGS_SYNERGY_FORCE_WEAPON_COLLISION);
        for (auto client : clients) threads.emplace_back([&, client] {
            for (unsigned i = 0; i != 1000; ++i) {
                HIGGS_SynergyLeaseInfo lease{};
                if (service.AcquireLease(client, request, 1000, lease) != HIGGS_SYNERGY_OK) { ++failures; continue; }
                HIGGS_SynergyRenewRequest renewal{ sizeof(renewal), 60000, lease.worldGeneration, lease.lease };
                if (service.RenewLease(client, renewal, 1000, lease) != HIGGS_SYNERGY_OK) ++failures;
                if (service.ReleaseLease(client, lease.lease, 1000) != HIGGS_SYNERGY_OK) ++failures;
            }
        });
        threads.emplace_back([&] {
            for (unsigned i = 0; i != 6000; ++i) {
                service.SetLegacyHandDisabled(true, i % 2 != 0);
                service.SetLegacyHandDisabled(true, false);
            }
        });
        threads.emplace_back([&] {
            std::uint64_t lastGeneration = 0;
            for (unsigned i = 0; i != 12000; ++i) {
                auto state = service.ReadEffectiveState(1000);
                if (!state.hands[0].handInteractionsDisabled || !state.hands[1].handInteractionsDisabled ||
                    state.hands[0].activeScopedLeases != state.hands[1].activeScopedLeases ||
                    state.stateGeneration < lastGeneration) ++failures;
                lastGeneration = state.stateGeneration;
            }
        });
        for (auto &thread : threads) thread.join();
        Check(failures == 0, "concurrent overlap, renewal and snapshots consistent");
        Check(service.UnregisterClient(baselineOwner, 1000) == HIGGS_SYNERGY_OK, "release baseline client");
        auto state = service.ReadEffectiveState(1000);
        Check(state.hands[0].activeScopedLeases == 0 && state.hands[1].activeScopedLeases == 0, "no lease leak after concurrent calls");
    }

    void ConcurrentClientRevocation() {
        RestrictionService service;
        service.ObserveWorld(1, 1000);
        auto owner = Client(service), observer = Client(service);
        auto request = Request(service, HIGGS_SYNERGY_HAND_BOTH, HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS);
        std::atomic<bool> start{false};
        std::atomic<unsigned> failures{0};
        std::thread worker([&] {
            while (!start.load()) std::this_thread::yield();
            for (unsigned i = 0; i < 1000; ++i) {
                HIGGS_SynergyLeaseInfo lease{};
                auto status = service.AcquireLease(owner, request, 1000, lease);
                if (status != HIGGS_SYNERGY_OK && status != HIGGS_SYNERGY_INVALID_CLIENT) ++failures;
                if (status == HIGGS_SYNERGY_OK) {
                    status = service.ReleaseLease(owner, lease.lease, 1000);
                    if (status != HIGGS_SYNERGY_OK && status != HIGGS_SYNERGY_INVALID_CLIENT) ++failures;
                }
            }
        });
        start = true;
        Check(service.UnregisterClient(owner, 1000) == HIGGS_SYNERGY_OK, "concurrent unregister");
        worker.join();
        HIGGS_SynergyEffectiveState state{};
        Check(failures == 0 && service.CopyEffectiveState(observer, 1000, state) == HIGGS_SYNERGY_OK &&
            state.hands[0].activeScopedLeases == 0 && state.hands[1].activeScopedLeases == 0, "unregister linearizes without stale authority");
    }

    void AbiNegotiationAndGuards() {
        HIGGS_SynergyInterface table{};
        std::memset(&table, 0xa5, sizeof(table));
        auto original = table;
        Check(HIGGS_QuerySynergyInterface(0, sizeof(table), &table) == HIGGS_SYNERGY_UNSUPPORTED_VERSION, "exact version negotiation");
        Check(std::memcmp(&table, &original, sizeof(table)) == 0, "no query output on version failure");
        Check(HIGGS_QuerySynergyInterface(1, sizeof(table) - 1, &table) == HIGGS_SYNERGY_BUFFER_TOO_SMALL, "undersized table refused");
        Check(std::memcmp(&table, &original, sizeof(table)) == 0, "no undersized table writes");
        Check(HIGGS_QuerySynergyInterface(1, sizeof(table), nullptr) == HIGGS_SYNERGY_INVALID_ARGUMENT, "null table refused");
        struct Extended { HIGGS_SynergyInterface table; std::uint64_t canary; } extended{};
        extended.canary = UINT64_C(0x0123456789abcdef);
        Check(HIGGS_QuerySynergyInterface(1, sizeof(extended), &extended.table) == HIGGS_SYNERGY_OK &&
            extended.canary == UINT64_C(0x0123456789abcdef), "copy touches known table bytes only");
        table = extended.table;
        Check(table.abiVersion == 1 && table.structSize == sizeof(table) && table.maxLeaseMs == 60000, "negotiated capabilities and limits");
        HIGGS_SynergyClientInfo info{ sizeof(info), 0, "ABI guard test" };
        HIGGS_SynergyClient client = 0;
        Check(table.RegisterClient(&info, &client) == HIGGS_SYNERGY_OK, "table registration");
        struct alignas(8) ShortRecord { std::uint32_t size; std::uint32_t canary[3]; } shortRecord{ 4, {11, 22, 33} };
        Check(table.CopyEffectiveState(client, reinterpret_cast<HIGGS_SynergyEffectiveState*>(&shortRecord)) ==
            HIGGS_SYNERGY_BUFFER_TOO_SMALL, "undersized snapshot reported");
        Check(shortRecord.size == sizeof(HIGGS_SynergyEffectiveState) && shortRecord.canary[0] == 11 &&
            shortRecord.canary[1] == 22 && shortRecord.canary[2] == 33, "only required-size field written");
        Check(table.CopyEffectiveState(client, nullptr) == HIGGS_SYNERGY_INVALID_ARGUMENT, "null record refused");
        auto malformed = info;
        malformed.structSize = 4;
        HIGGS_SynergyClient invalid = 999;
        Check(table.RegisterClient(&malformed, &invalid) == HIGGS_SYNERGY_INVALID_ARGUMENT && invalid == 0, "undersized client record refused");
        malformed = info;
        std::memset(malformed.name, 'x', sizeof(malformed.name));
        Check(table.RegisterClient(&malformed, &invalid) == HIGGS_SYNERGY_INVALID_ARGUMENT, "unterminated copied name refused");
        Check(table.UnregisterClient(client) == HIGGS_SYNERGY_OK, "table unregister");
        HiggsSynergy::ObserveWorld(77);
        Check(synergy_c_consumer(HIGGS_QuerySynergyInterface) == 0, "real C consumer through copied table");
        HiggsSynergy::ObserveWorld(0);
    }
}

int main() {
    const struct { const char* name; void (*run)(); } tests[] = {
        { "overlap, legacy isolation and possession", OverlapLegacyAndPossession },
        { "collision precedence and removable force", CollisionPrecedenceAndRemovableForce },
        { "renewal, expiry and clock ordering", RenewalExpiryAndClockOrdering },
        { "capacity and all-or-none requests", CapacityAndAllOrNone },
        { "argument validation and checked time", ValidationIsNonMutating },
        { "world and game lifecycle", WorldAndGameLifecycle },
        { "concurrent owners and coherent snapshots", ConcurrentOwnersAndSnapshots },
        { "concurrent client revocation", ConcurrentClientRevocation },
        { "C ABI negotiation and buffer guards", AbiNegotiationAndGuards }
    };
    try {
        for (const auto &test : tests) { test.run(); std::cout << "PASS " << test.name << '\n'; }
    }
    catch (const std::exception &error) { std::cerr << "FAIL " << error.what() << '\n'; return 1; }
    std::cout << "All production-core and ABI tests passed.\n";
}
