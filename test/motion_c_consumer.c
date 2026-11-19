#include "higgsobservations.h"
#include <stddef.h>
#include <string.h>

_Static_assert(sizeof(HIGGS_MotionHand) == 96, "C hand layout");
_Static_assert(sizeof(HIGGS_MotionSnapshot) == 232, "C pair layout");
_Static_assert(sizeof(HIGGS_MotionInterface) == 32, "C x64 table layout");
_Static_assert(offsetof(HIGGS_MotionHand, latestLinear) == 40, "C vector offset");
_Static_assert(offsetof(HIGGS_MotionSnapshot, hands) == 40, "C pair offset");
_Static_assert(offsetof(HIGGS_MotionInterface, CopyVelocitySnapshot) == 24, "C function offset");

/* Example consumer policy: a REQUIRED caller-chosen limit, never an ABI default.
   Numeric zero alone is not admitted; neither unknown age nor unavailable hands are. */
int motion_c_accepts(const HIGGS_MotionSnapshot* state, uint32_t side, uint64_t maximumAgeNs) {
    const HIGGS_MotionHand* hand;
    if (side != HIGGS_MOTION_LEFT && side != HIGGS_MOTION_RIGHT) return 0;
    hand = &state->hands[side == HIGGS_MOTION_LEFT ? 0 : 1];
    return (state->availableHands & side) && hand->status == HIGGS_MOTION_VALID &&
        (hand->validFields & (HIGGS_MOTION_AGE_KNOWN | HIGGS_MOTION_LINEAR_VALID)) ==
        (HIGGS_MOTION_AGE_KNOWN | HIGGS_MOTION_LINEAR_VALID) && hand->ageNs <= maximumAgeNs;
}

int motion_c_consumer(HIGGS_QueryMotionInterfaceFn query) {
    HIGGS_MotionInterface api;
    HIGGS_MotionSnapshot state = {0};
    memset(&api, 0, sizeof(api));
    if (query(1, sizeof(api), &api) != HIGGS_MOTION_OK || api.structSize != sizeof(api) ||
        api.abiVersion != 1 || api.historySize != 5 || api.capabilities != UINT64_C(15) || api.reserved) return 1;
    state.structSize = sizeof(state);
    if (!api.CopyVelocitySnapshot || api.CopyVelocitySnapshot(&state) != HIGGS_MOTION_OK ||
        state.structSize != sizeof(state) || state.abiVersion != 1 || state.availableHands != HIGGS_MOTION_BOTH) return 2;
    if (state.hands[0].physicalHand != HIGGS_MOTION_LEFT || state.hands[1].physicalHand != HIGGS_MOTION_RIGHT ||
        state.hands[0].latestLinear.x != 1 || state.hands[0].latestLinear.y != -3 || state.hands[0].latestLinear.z != 2 ||
        state.hands[0].latestAngular.x != 4 || state.hands[0].latestAngular.y != -6 || state.hands[0].latestAngular.z != 5) return 3;
    if (state.hands[0].meanLinear.y != -3 || state.hands[0].peakLinear.z != 2 ||
        state.hands[0].meanSpeed < 3.7f || state.hands[0].meanSpeed > 3.8f ||
        state.hands[1].meanSpeed != 0 || state.hands[1].latestLinear.x != 0) return 4;
    if (!state.hands[0].acceptedObservation || state.hands[0].acceptedObservation != state.hands[1].acceptedObservation ||
        state.hands[0].sourceFlags != (HIGGS_MOTION_SOURCE_POSE_CALLBACK | HIGGS_MOTION_NO_HARDWARE_TIMESTAMP) ||
        state.hands[0].historyObservationCount != 5 || state.hands[1].historyObservationCount != 5) return 5;
    if (!motion_c_accepts(&state, HIGGS_MOTION_RIGHT, UINT64_MAX)) return 6; /* Valid stationary sample. */
    return 0;
}
