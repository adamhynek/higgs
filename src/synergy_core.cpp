#include "synergy_core.h"

#include <algorithm>
#include <cstring>
#include <limits>

namespace HiggsSynergy {
    namespace {
        constexpr std::uint32_t allFlags = HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS |
            HIGGS_SYNERGY_SUPPRESS_WEAPON_COLLISION | HIGGS_SYNERGY_FORCE_WEAPON_COLLISION;
        HIGGS_SynergyLeaseInfo LeaseInfo(HIGGS_SynergyLease token, std::uint64_t world, std::uint64_t expiry) {
            return { sizeof(HIGGS_SynergyLeaseInfo), 0, token, world, expiry };
        }
    }

    bool RestrictionService::HasClientLocked(HIGGS_SynergyClient client) const {
        return client != 0 && std::any_of(clients_.begin(), clients_.end(),
            [client](const ClientSlot &slot) { return slot.token == client; });
    }

    RestrictionService::LeaseSlot* RestrictionService::FindLeaseLocked(HIGGS_SynergyLease lease) {
        if (!lease) return nullptr;
        for (auto &slot : leases_) if (slot.token == lease) return &slot;
        return nullptr;
    }

    void RestrictionService::ExpireLocked(std::uint64_t nowMs) {
        // Concurrent callers can arrive in a different order than their clock samples.
        nowMs_ = (std::max)(nowMs_, nowMs);
        bool changed = false;
        for (auto &slot : leases_) {
            if (slot.token && slot.expiresAtMs <= nowMs_) {
                slot = {};
                changed = true;
            }
        }
        if (changed) ++stateGeneration_;
    }

    bool RestrictionService::ValidDurationLocked(std::uint32_t durationMs) const {
        return durationMs >= HIGGS_SYNERGY_MIN_LEASE_MS && durationMs <= HIGGS_SYNERGY_MAX_LEASE_MS &&
            nowMs_ <= (std::numeric_limits<std::uint64_t>::max)() - durationMs;
    }

    HIGGS_SynergyStatus RestrictionService::RegisterClient(const HIGGS_SynergyClientInfo &info,
        std::uint64_t nowMs, HIGGS_SynergyClient &outClient) {
        outClient = 0;
        if (info.structSize < sizeof(info) || info.reserved || !info.name[0] ||
            !std::memchr(info.name, '\0', sizeof(info.name))) return HIGGS_SYNERGY_INVALID_ARGUMENT;
        std::scoped_lock lock(mutex_);
        ExpireLocked(nowMs);
        if (!nextClient_) return HIGGS_SYNERGY_CAPACITY;
        for (auto &slot : clients_) {
            if (!slot.token) {
                slot = { nextClient_++, info };
                outClient = slot.token;
                ++stateGeneration_;
                return HIGGS_SYNERGY_OK;
            }
        }
        return HIGGS_SYNERGY_CAPACITY;
    }

    HIGGS_SynergyStatus RestrictionService::UnregisterClient(HIGGS_SynergyClient client, std::uint64_t nowMs) {
        std::scoped_lock lock(mutex_);
        ExpireLocked(nowMs);
        if (!HasClientLocked(client)) return HIGGS_SYNERGY_INVALID_CLIENT;
        for (auto &slot : clients_) if (slot.token == client) slot = {};
        for (auto &slot : leases_) if (slot.owner == client) slot = {};
        ++stateGeneration_;
        return HIGGS_SYNERGY_OK;
    }

    HIGGS_SynergyStatus RestrictionService::AcquireLease(HIGGS_SynergyClient client,
        const HIGGS_SynergyLeaseRequest &request, std::uint64_t nowMs, HIGGS_SynergyLeaseInfo &outLease) {
        outLease = LeaseInfo(0, 0, 0);
        if (request.structSize < sizeof(request) || !request.physicalHands ||
            (request.physicalHands & ~HIGGS_SYNERGY_HAND_BOTH) || !request.flags || (request.flags & ~allFlags))
            return HIGGS_SYNERGY_INVALID_ARGUMENT;
        std::scoped_lock lock(mutex_);
        ExpireLocked(nowMs);
        if (!HasClientLocked(client)) return HIGGS_SYNERGY_INVALID_CLIENT;
        if (!ValidDurationLocked(request.durationMs)) return HIGGS_SYNERGY_INVALID_ARGUMENT;
        if (!worldReady_) return HIGGS_SYNERGY_NOT_READY;
        if (request.expectedWorldGeneration != worldGeneration_) return HIGGS_SYNERGY_STALE_WORLD;
        if (!nextLease_) return HIGGS_SYNERGY_CAPACITY;
        for (auto &slot : leases_) {
            if (!slot.token) {
                slot = { nextLease_++, client, nowMs_ + request.durationMs, request.physicalHands, request.flags };
                outLease = LeaseInfo(slot.token, worldGeneration_, slot.expiresAtMs);
                ++stateGeneration_;
                return HIGGS_SYNERGY_OK;
            }
        }
        return HIGGS_SYNERGY_CAPACITY;
    }

    HIGGS_SynergyStatus RestrictionService::RenewLease(HIGGS_SynergyClient client,
        const HIGGS_SynergyRenewRequest &request, std::uint64_t nowMs, HIGGS_SynergyLeaseInfo &outLease) {
        outLease = LeaseInfo(0, 0, 0);
        if (request.structSize < sizeof(request)) return HIGGS_SYNERGY_INVALID_ARGUMENT;
        std::scoped_lock lock(mutex_);
        ExpireLocked(nowMs);
        if (!HasClientLocked(client)) return HIGGS_SYNERGY_INVALID_CLIENT;
        if (!ValidDurationLocked(request.durationMs)) return HIGGS_SYNERGY_INVALID_ARGUMENT;
        if (!worldReady_) return HIGGS_SYNERGY_NOT_READY;
        if (request.expectedWorldGeneration != worldGeneration_) return HIGGS_SYNERGY_STALE_WORLD;
        auto slot = FindLeaseLocked(request.lease);
        if (!slot) return HIGGS_SYNERGY_INVALID_LEASE;
        if (slot->owner != client) return HIGGS_SYNERGY_WRONG_OWNER;
        slot->expiresAtMs = nowMs_ + request.durationMs;
        outLease = LeaseInfo(slot->token, worldGeneration_, slot->expiresAtMs);
        ++stateGeneration_;
        return HIGGS_SYNERGY_OK;
    }

    HIGGS_SynergyStatus RestrictionService::ReleaseLease(HIGGS_SynergyClient client,
        HIGGS_SynergyLease lease, std::uint64_t nowMs) {
        std::scoped_lock lock(mutex_);
        ExpireLocked(nowMs);
        if (!HasClientLocked(client)) return HIGGS_SYNERGY_INVALID_CLIENT;
        auto slot = FindLeaseLocked(lease);
        if (!slot) return HIGGS_SYNERGY_INVALID_LEASE;
        if (slot->owner != client) return HIGGS_SYNERGY_WRONG_OWNER;
        *slot = {};
        ++stateGeneration_;
        return HIGGS_SYNERGY_OK;
    }

    HIGGS_SynergyEffectiveState RestrictionService::SnapshotLocked() const {
        HIGGS_SynergyEffectiveState state{};
        state.structSize = sizeof(state);
        state.worldReady = worldReady_;
        state.stateGeneration = stateGeneration_;
        state.worldGeneration = worldGeneration_;
        state.sampledAtMs = nowMs_;
        for (std::size_t i = 0; i != 2; ++i) {
            // Explicit [0]=physical left, [1]=physical right, independent of game handedness.
            auto &hand = state.hands[i];
            const auto &legacy = legacy_[i];
            hand.physicalHand = i == 0 ? HIGGS_SYNERGY_HAND_LEFT : HIGGS_SYNERGY_HAND_RIGHT;
            hand.legacyHandDisabled = legacy.handDisabled;
            hand.legacyWeaponCollisionDisabled = legacy.collisionDisabled;
            hand.legacyForceWeaponCollisionEnabled = legacy.forceCollision;
            for (const auto &slot : leases_) {
                if (slot.token && (slot.hands & hand.physicalHand)) {
                    hand.scopedFlags |= slot.flags;
                    ++hand.activeScopedLeases;
                }
            }
            hand.handInteractionsDisabled = legacy.handDisabled ||
                (hand.scopedFlags & HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS);
            hand.weaponCollisionDisabled = legacy.collisionDisabled ||
                (hand.scopedFlags & HIGGS_SYNERGY_SUPPRESS_WEAPON_COLLISION);
            hand.forceWeaponCollisionEnabled = !hand.weaponCollisionDisabled &&
                (legacy.forceCollision || (hand.scopedFlags & HIGGS_SYNERGY_FORCE_WEAPON_COLLISION));
        }
        return state;
    }

    HIGGS_SynergyStatus RestrictionService::CopyEffectiveState(HIGGS_SynergyClient client,
        std::uint64_t nowMs, HIGGS_SynergyEffectiveState &outState) {
        std::scoped_lock lock(mutex_);
        ExpireLocked(nowMs);
        if (!HasClientLocked(client)) return HIGGS_SYNERGY_INVALID_CLIENT;
        outState = SnapshotLocked();
        return HIGGS_SYNERGY_OK;
    }

    HIGGS_SynergyEffectiveState RestrictionService::ReadEffectiveState(std::uint64_t nowMs) {
        std::scoped_lock lock(mutex_);
        ExpireLocked(nowMs);
        return SnapshotLocked();
    }

    void RestrictionService::SetLegacyHandDisabled(bool isLeft, bool disabled) {
        std::scoped_lock lock(mutex_);
        auto &value = legacy_[isLeft ? 0 : 1].handDisabled;
        if (value != disabled) { value = disabled; ++stateGeneration_; }
    }

    void RestrictionService::SetLegacyWeaponCollisionDisabled(bool isLeft, bool disabled) {
        std::scoped_lock lock(mutex_);
        auto &value = legacy_[isLeft ? 0 : 1].collisionDisabled;
        if (value != disabled) { value = disabled; ++stateGeneration_; }
    }

    void RestrictionService::ForceLegacyWeaponCollision(bool isLeft) {
        std::scoped_lock lock(mutex_);
        auto &value = legacy_[isLeft ? 0 : 1].forceCollision;
        if (!value) { value = true; ++stateGeneration_; }
    }

    void RestrictionService::ResetWorldLocked() {
        for (auto &slot : leases_) slot = {};
        worldIdentity_ = 0;
        worldReady_ = false;
        ++worldGeneration_;
        ++stateGeneration_;
    }

    void RestrictionService::ObserveWorld(std::uintptr_t identity, std::uint64_t nowMs) {
        std::scoped_lock lock(mutex_);
        ExpireLocked(nowMs);
        if (gameLoading_) return;
        if (identity != worldIdentity_ || (identity && !worldReady_)) {
            ResetWorldLocked();
            worldIdentity_ = identity;
            worldReady_ = identity != 0;
        }
    }

    void RestrictionService::BeginGameLoad() {
        std::scoped_lock lock(mutex_);
        gameLoading_ = true;
        ResetWorldLocked();
    }

    void RestrictionService::EndGameLoad() {
        std::scoped_lock lock(mutex_);
        gameLoading_ = false;
    }

    void RestrictionService::NewGame() {
        std::scoped_lock lock(mutex_);
        gameLoading_ = false;
        ResetWorldLocked();
    }
}
