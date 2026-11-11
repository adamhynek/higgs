#pragma once
#include "higgssynergy.h"

namespace HiggsSynergy {
    inline bool MayStartHandInteraction(const HIGGS_SynergyHandState &state) {
        return state.handInteractionsDisabled == 0;
    }

    // Scoped admission restrictions do not release existing possession.
    // The old DisableHand API retains its existing forced-drop behavior.
    inline bool MayContinueHeldInteraction(const HIGGS_SynergyHandState &state) {
        return state.legacyHandDisabled == 0;
    }

    inline bool ShouldDisableWeaponCollision(const HIGGS_SynergyHandState &state,
        bool weaponDrawn, bool withinHitDisableWindow) {
        return !weaponDrawn || state.weaponCollisionDisabled != 0 ||
            (state.forceWeaponCollisionEnabled == 0 && withinHitDisableWindow);
    }
}
