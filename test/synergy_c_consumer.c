#include "higgsphysics.h"
#include <stddef.h>
#include <string.h>

_Static_assert(sizeof(HIGGS_SynergyClientInfo) == 72, "client layout");
_Static_assert(sizeof(HIGGS_SynergyLeaseRequest) == 24, "request layout");
_Static_assert(sizeof(HIGGS_SynergyRenewRequest) == 24, "renew layout");
_Static_assert(sizeof(HIGGS_SynergyLeaseInfo) == 32, "lease layout");
_Static_assert(sizeof(HIGGS_SynergyHandState) == 36, "hand layout");
_Static_assert(sizeof(HIGGS_SynergyEffectiveState) == 104, "snapshot layout");
_Static_assert(offsetof(HIGGS_SynergyInterface, RegisterClient) == 32, "table prefix");
_Static_assert(sizeof(HIGGS_SynergyInterface) == 80, "x64 table layout");

int synergy_c_consumer(HIGGS_QuerySynergyInterfaceFn query) {
    HIGGS_SynergyInterface table;
    HIGGS_SynergyClientInfo info = { sizeof(info), 0, "standalone C consumer" };
    HIGGS_SynergyClient client = 0;
    HIGGS_SynergyEffectiveState state = {0};
    HIGGS_SynergyLeaseRequest request = {0};
    HIGGS_SynergyLeaseInfo lease = {0};
    state.structSize = sizeof(state);
    lease.structSize = sizeof(lease);
    if (query(HIGGS_SYNERGY_ABI_VERSION, sizeof(table), &table) != HIGGS_SYNERGY_OK) return 1;
    if (table.RegisterClient(&info, &client) != HIGGS_SYNERGY_OK || client == 0) return 2;
    if (table.CopyEffectiveState(client, &state) != HIGGS_SYNERGY_OK || !state.worldReady) return 3;
    request.structSize = sizeof(request);
    request.physicalHands = HIGGS_SYNERGY_HAND_LEFT;
    request.flags = HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS;
    request.durationMs = 60000;
    request.expectedWorldGeneration = state.worldGeneration;
    if (table.AcquireLease(client, &request, &lease) != HIGGS_SYNERGY_OK) return 4;
    if (table.CopyEffectiveState(client, &state) != HIGGS_SYNERGY_OK || !state.hands[0].handInteractionsDisabled ||
        state.hands[1].handInteractionsDisabled) return 5;
    if (table.ReleaseLease(client, lease.lease) != HIGGS_SYNERGY_OK) return 6;
    if (table.UnregisterClient(client) != HIGGS_SYNERGY_OK) return 7;
    return 0;
}
