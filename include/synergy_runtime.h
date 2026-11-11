#pragma once
#include "higgssynergy.h"
#include "synergy_policy.h"
#include <cstdint>

namespace HiggsSynergy {
    HIGGS_SynergyHandState CopyHandState(bool isLeft);
    void SetLegacyHandDisabled(bool isLeft, bool disabled);
    void SetLegacyWeaponCollisionDisabled(bool isLeft, bool disabled);
    void ForceLegacyWeaponCollision(bool isLeft);
    void ObserveWorld(std::uintptr_t identity);
    void BeginGameLoad();
    void EndGameLoad();
    void NewGame();
}
