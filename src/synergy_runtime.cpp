#include "synergy_runtime.h"
#include "synergy_core.h"
#include "hand_broker.h"

#include <chrono>
#include <cstring>

namespace {
    HiggsSynergy::RestrictionService& Service() {
        static HiggsSynergy::RestrictionService service;
        return service;
    }

    std::uint64_t NowMs() {
        return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    }

    template<class Record>
    HIGGS_SynergyStatus CheckOutput(Record* output) {
        if (!output) return HIGGS_SYNERGY_INVALID_ARGUMENT;
        if (output->structSize < sizeof(Record)) {
            output->structSize = sizeof(Record);
            return HIGGS_SYNERGY_BUFFER_TOO_SMALL;
        }
        return HIGGS_SYNERGY_OK;
    }

    template<class Operation>
    HIGGS_SynergyStatus Guard(Operation operation) noexcept {
        try { return operation(); }
        catch (...) { return HIGGS_SYNERGY_INTERNAL_ERROR; }
    }

    HIGGS_SynergyStatus HIGGS_SYNERGY_CALL RegisterClient(const HIGGS_SynergyClientInfo* info, HIGGS_SynergyClient* outClient) {
        if (!outClient) return HIGGS_SYNERGY_INVALID_ARGUMENT;
        *outClient = 0;
        if (!info || info->structSize < sizeof(*info)) return HIGGS_SYNERGY_INVALID_ARGUMENT;
        return Guard([&] { return Service().RegisterClient(*info, NowMs(), *outClient); });
    }

    HIGGS_SynergyStatus HIGGS_SYNERGY_CALL UnregisterClient(HIGGS_SynergyClient client) {
        return Guard([&] { return Service().UnregisterClient(client, NowMs()); });
    }

    HIGGS_SynergyStatus HIGGS_SYNERGY_CALL AcquireLease(HIGGS_SynergyClient client,
        const HIGGS_SynergyLeaseRequest* request, HIGGS_SynergyLeaseInfo* outLease) {
        if (!request || request->structSize < sizeof(*request)) return HIGGS_SYNERGY_INVALID_ARGUMENT;
        auto status = CheckOutput(outLease);
        if (status != HIGGS_SYNERGY_OK) return status;
        return Guard([&] { return Service().AcquireLease(client, *request, NowMs(), *outLease); });
    }

    HIGGS_SynergyStatus HIGGS_SYNERGY_CALL RenewLease(HIGGS_SynergyClient client,
        const HIGGS_SynergyRenewRequest* request, HIGGS_SynergyLeaseInfo* outLease) {
        if (!request || request->structSize < sizeof(*request)) return HIGGS_SYNERGY_INVALID_ARGUMENT;
        auto status = CheckOutput(outLease);
        if (status != HIGGS_SYNERGY_OK) return status;
        return Guard([&] { return Service().RenewLease(client, *request, NowMs(), *outLease); });
    }

    HIGGS_SynergyStatus HIGGS_SYNERGY_CALL ReleaseLease(HIGGS_SynergyClient client, HIGGS_SynergyLease lease) {
        return Guard([&] { return Service().ReleaseLease(client, lease, NowMs()); });
    }

    HIGGS_SynergyStatus HIGGS_SYNERGY_CALL CopyEffectiveState(HIGGS_SynergyClient client, HIGGS_SynergyEffectiveState* outState) {
        auto status = CheckOutput(outState);
        if (status != HIGGS_SYNERGY_OK) return status;
        return Guard([&] { return Service().CopyEffectiveState(client, NowMs(), *outState); });
    }
}

extern "C" HIGGS_SynergyStatus HIGGS_SYNERGY_CALL HIGGS_QuerySynergyInterface(
    uint32_t version, uint32_t tableSize, HIGGS_SynergyInterface* outInterface) {
    if (version != HIGGS_SYNERGY_ABI_VERSION) return HIGGS_SYNERGY_UNSUPPORTED_VERSION;
    if (!outInterface) return HIGGS_SYNERGY_INVALID_ARGUMENT;
    if (tableSize < sizeof(HIGGS_SynergyInterface)) return HIGGS_SYNERGY_BUFFER_TOO_SMALL;
    // Copy only known v1 bytes, leaving any caller-owned extension untouched.
    const HIGGS_SynergyInterface table{
        sizeof(HIGGS_SynergyInterface), HIGGS_SYNERGY_ABI_VERSION,
        HIGGS_SYNERGY_CAP_SCOPED_RESTRICTIONS, HIGGS_SYNERGY_MAX_CLIENTS, HIGGS_SYNERGY_MAX_LEASES,
        HIGGS_SYNERGY_MIN_LEASE_MS, HIGGS_SYNERGY_MAX_LEASE_MS,
        RegisterClient, UnregisterClient, AcquireLease, RenewLease, ReleaseLease, CopyEffectiveState
    };
    std::memcpy(outInterface, &table, sizeof(table));
    return HIGGS_SYNERGY_OK;
}

namespace HiggsSynergy {
    HIGGS_SynergyHandState CopyHandState(bool isLeft) {
        return Service().ReadEffectiveState(NowMs()).hands[isLeft ? 0 : 1];
    }
    void SetLegacyHandDisabled(bool isLeft, bool disabled) { Service().SetLegacyHandDisabled(isLeft, disabled); }
    void SetLegacyWeaponCollisionDisabled(bool isLeft, bool disabled) { Service().SetLegacyWeaponCollisionDisabled(isLeft, disabled); }
    void ForceLegacyWeaponCollision(bool isLeft) { Service().ForceLegacyWeaponCollision(isLeft); }
    void ObserveWorld(std::uintptr_t identity) { Service().ObserveWorld(identity, NowMs()); HiggsHands::ObserveWorld(identity); }
    void BeginGameLoad() { Service().BeginGameLoad(); HiggsHands::BeginLoad(); }
    void EndGameLoad() { Service().EndGameLoad(); HiggsHands::EndLoad(); }
    void NewGame() { Service().NewGame(); HiggsHands::NewGame(); }
}
