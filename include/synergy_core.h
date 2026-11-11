#pragma once
#include "higgssynergy.h"

#include <array>
#include <cstdint>
#include <mutex>

namespace HiggsSynergy {
    // Production state machine, independent of SKSE/Havok. All methods serialize;
    // no engine call, callback, allocation or caller-owned pointer is retained.
    class RestrictionService {
    public:
        HIGGS_SynergyStatus RegisterClient(const HIGGS_SynergyClientInfo&, std::uint64_t nowMs, HIGGS_SynergyClient&);
        HIGGS_SynergyStatus UnregisterClient(HIGGS_SynergyClient, std::uint64_t nowMs);
        HIGGS_SynergyStatus AcquireLease(HIGGS_SynergyClient, const HIGGS_SynergyLeaseRequest&, std::uint64_t nowMs, HIGGS_SynergyLeaseInfo&);
        HIGGS_SynergyStatus RenewLease(HIGGS_SynergyClient, const HIGGS_SynergyRenewRequest&, std::uint64_t nowMs, HIGGS_SynergyLeaseInfo&);
        HIGGS_SynergyStatus ReleaseLease(HIGGS_SynergyClient, HIGGS_SynergyLease, std::uint64_t nowMs);
        HIGGS_SynergyStatus CopyEffectiveState(HIGGS_SynergyClient, std::uint64_t nowMs, HIGGS_SynergyEffectiveState&);
        HIGGS_SynergyEffectiveState ReadEffectiveState(std::uint64_t nowMs);

        void SetLegacyHandDisabled(bool isLeft, bool disabled);
        void SetLegacyWeaponCollisionDisabled(bool isLeft, bool disabled);
        void ForceLegacyWeaponCollision(bool isLeft);
        void ObserveWorld(std::uintptr_t identity, std::uint64_t nowMs);
        void BeginGameLoad();
        void EndGameLoad();
        void NewGame();

    private:
        struct ClientSlot {
            HIGGS_SynergyClient token = 0;
            HIGGS_SynergyClientInfo info{};
        };
        struct LeaseSlot {
            HIGGS_SynergyLease token = 0;
            HIGGS_SynergyClient owner = 0;
            std::uint64_t expiresAtMs = 0;
            std::uint32_t hands = 0;
            std::uint32_t flags = 0;
        };
        struct LegacyState {
            bool handDisabled = false;
            bool collisionDisabled = false;
            bool forceCollision = false;
        };

        bool HasClientLocked(HIGGS_SynergyClient) const;
        LeaseSlot* FindLeaseLocked(HIGGS_SynergyLease);
        void ExpireLocked(std::uint64_t nowMs);
        void ResetWorldLocked();
        HIGGS_SynergyEffectiveState SnapshotLocked() const;
        bool ValidDurationLocked(std::uint32_t durationMs) const;

        std::mutex mutex_;
        std::array<ClientSlot, HIGGS_SYNERGY_MAX_CLIENTS> clients_{};
        std::array<LeaseSlot, HIGGS_SYNERGY_MAX_LEASES> leases_{};
        std::array<LegacyState, 2> legacy_{};
        HIGGS_SynergyClient nextClient_ = 1;
        HIGGS_SynergyLease nextLease_ = 1;
        std::uint64_t stateGeneration_ = 1;
        std::uint64_t worldGeneration_ = 1;
        std::uint64_t nowMs_ = 0;
        std::uintptr_t worldIdentity_ = 0;
        bool worldReady_ = false;
        bool gameLoading_ = false;
    };
}
