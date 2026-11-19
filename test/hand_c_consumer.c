#include "higgshandapi.h"
#include "higgsphysics.h"
#include "higgstransactions.h"
#include "higgsinput.h"
#include <stddef.h>
#include <string.h>
_Static_assert(sizeof(HIGGS_HandRecord) == 120, "hand record x64 ABI");
_Static_assert(sizeof(HIGGS_NativeRegionFrame)==184,"native region frame ABI");
_Static_assert(sizeof(HIGGS_NativeRegionResult)==24,"native region result ABI");
_Static_assert(sizeof(HIGGS_NativeRegionsInterface)==40,"native region table ABI");
_Static_assert(sizeof(HIGGS_HandUseTransitionRequest)==56,"hand transition request ABI");
_Static_assert(sizeof(HIGGS_HandUseTransitionState)==136,"hand transition state ABI");
_Static_assert(sizeof(HIGGS_HandUseTransitionInterface)==32,"hand transition table ABI");
_Static_assert(sizeof(HIGGS_HandUseProposalRequest)==72,"proposal request ABI");
_Static_assert(sizeof(HIGGS_HandUseProposalState)==160,"proposal state ABI");
_Static_assert(sizeof(HIGGS_HandUseProposalsInterface)==40,"proposal table ABI");
_Static_assert(sizeof(HIGGS_HandSnapshot) == 280, "hand snapshot x64 ABI");
_Static_assert(offsetof(HIGGS_HandSnapshot, hands) == 40, "hand pair offset");
_Static_assert(sizeof(HIGGS_HandInterface) == 120, "hand table x64 ABI");
_Static_assert(sizeof(HIGGS_HandRoutingInterface) == 48, "routing table x64 ABI");
_Static_assert(sizeof(HIGGS_HandOperationsInterface) == 40, "operations table x64 ABI");
_Static_assert(sizeof(HIGGS_HandOperation) == 136, "operation request x64 ABI");
_Static_assert(sizeof(HIGGS_HandOperationState) == 224, "operation state x64 ABI");
_Static_assert(sizeof(HIGGS_HandInput) == 336, "input observation x64 ABI");
_Static_assert(sizeof(HIGGS_TransactionsInterface) == 80, "transaction table ABI");
_Static_assert(sizeof(HIGGS_ItemIdentity) == 72, "item identity ABI");
_Static_assert(sizeof(HIGGS_TransferRequest) == 160, "transfer request ABI");
_Static_assert(sizeof(HIGGS_TransferState) == 312, "transfer state ABI");
_Static_assert(sizeof(HIGGS_TransferReceipt) == 288, "transfer receipt ABI");
_Static_assert(sizeof(HIGGS_GrantMutationInterface) == 16, "mutation table ABI");
_Static_assert(sizeof(HIGGS_ActionInventoryEffect) == 32, "inventory effect record ABI");
_Static_assert(offsetof(HIGGS_ActionInventoryEffect, parentRequestId) == 8, "inventory effect parent offset");
_Static_assert(offsetof(HIGGS_ActionInventoryEffect, childCount) == 24, "inventory effect count offset");
_Static_assert(sizeof(HIGGS_InventoryEffectsInterface) == 24, "inventory effect table ABI");
_Static_assert(offsetof(HIGGS_InventoryEffectsInterface, CopyForAction) == 16, "inventory effect function offset");
_Static_assert(HIGGS_INVENTORY_EFFECT_CLOSED == 1 && HIGGS_INVENTORY_EFFECT_COMMITTED == 2 &&
    HIGGS_INVENTORY_EFFECT_UNCERTAIN == 4 && HIGGS_INVENTORY_EFFECT_RESTORED == 8, "inventory effect flags");
_Static_assert(HIGGS_HAND_CAP_ACCEPTED_RECEIPTS == 64, "queued receipt capability");
_Static_assert(sizeof(HIGGS_AuthorizedDispatchRequest) == 40, "authorized dispatch request ABI");
_Static_assert(offsetof(HIGGS_AuthorizedDispatchRequest, grant) == 32, "authorized dispatch grant offset");
_Static_assert(sizeof(HIGGS_AuthorizedDispatchInterface) == 16, "authorized dispatch table ABI");
_Static_assert(sizeof(HIGGS_SupportInterface) == 48, "support table ABI");
_Static_assert(sizeof(HIGGS_SupportRequest) == 168, "support request ABI");
_Static_assert(sizeof(HIGGS_SupportState) == 336, "support state ABI");
_Static_assert(sizeof(HIGGS_SupportUpdate) == 72, "support update ABI");
int hand_c_consumer(void)
{
    HIGGS_InventoryEffectsInterface effects, original;
    memset(&effects, 0xa5, sizeof(effects));
    original = effects;
    if (HIGGS_QueryInventoryEffectsInterface(2, sizeof(effects), &effects) != HIGGS_HAND_UNSUPPORTED_VERSION ||
        memcmp(&effects, &original, sizeof(effects)) ||
        HIGGS_QueryInventoryEffectsInterface(1, sizeof(effects)-1, &effects) != HIGGS_HAND_BUFFER_TOO_SMALL ||
        memcmp(&effects, &original, sizeof(effects)) ||
        HIGGS_QueryInventoryEffectsInterface(1, sizeof(effects), NULL) != HIGGS_HAND_INVALID_ARGUMENT) return 7;
    struct ExtendedEffects { HIGGS_InventoryEffectsInterface table; uint64_t tail; } extended = {0};
    extended.tail = UINT64_C(0x12345678abcdef01);
    if (HIGGS_QueryInventoryEffectsInterface(1, sizeof(extended), &extended.table) ||
        extended.table.structSize != sizeof(effects) || extended.table.abiVersion != 1 ||
        extended.table.capabilities != 1 || !extended.table.CopyForAction ||
        extended.tail != UINT64_C(0x12345678abcdef01)) return 8;
    HIGGS_ActionInventoryEffect effect = {0}, previous;
    effect.structSize = sizeof(effect);
    effect.flags = 0x1234;
    previous = effect;
    if (extended.table.CopyForAction(0, 1, &effect) != HIGGS_HAND_INVALID_CLIENT ||
        memcmp(&effect, &previous, sizeof(effect)) ||
        extended.table.CopyForAction(0, 1, NULL) != HIGGS_HAND_INVALID_ARGUMENT) return 9;
    effect.structSize = sizeof(effect)-1;
    if (extended.table.CopyForAction(0, 1, &effect) != HIGGS_HAND_BUFFER_TOO_SMALL ||
        effect.structSize != sizeof(effect) || effect.flags != previous.flags ||
        effect.parentRequestId != previous.parentRequestId || effect.epoch != previous.epoch ||
        effect.childCount != previous.childCount || effect.reserved != previous.reserved) return 10;
    HIGGS_AuthorizedDispatchInterface dispatch = {0}, untouched = {0};
    if (HIGGS_QueryAuthorizedDispatchInterface(2, sizeof(dispatch), &dispatch) || !dispatch.Invoke ||
        dispatch.structSize != sizeof(dispatch) || dispatch.abiVersion != 2 ||
        HIGGS_QueryAuthorizedDispatchInterface(1, sizeof(untouched), &untouched) != HIGGS_HAND_UNSUPPORTED_VERSION ||
        HIGGS_QueryAuthorizedDispatchInterface(2, sizeof(untouched)-1, &untouched) != HIGGS_HAND_BUFFER_TOO_SMALL ||
        untouched.structSize || untouched.Invoke ||
        dispatch.Invoke(0, NULL, NULL, NULL) != HIGGS_HAND_INVALID_ARGUMENT) return 6;
    const HIGGS_NativeRegionsInterface* regions=HIGGS_QueryNativeRegionsInterface(1,sizeof(HIGGS_NativeRegionsInterface));
    if (!regions || !regions->RegisterDetector || !regions->CopyFrame ||
        HIGGS_QueryNativeRegionsInterface(2,sizeof(*regions)) || HIGGS_QueryNativeRegionsInterface(1,sizeof(*regions)-1)) return 5;
    HIGGS_HandInterface api = {0};
    HIGGS_HandRoutingInterface routing = {0};
    HIGGS_HandOperationsInterface operations = {0};
    if (HIGGS_QueryHandInterface(2, sizeof(api), &api))
        return 1;
    if (!(api.capabilities & HIGGS_HAND_CAP_ACCEPTED_RECEIPTS)) return 11;
    if (HIGGS_QueryHandRoutingInterface(2, sizeof(routing), &routing))
        return 2;
    if (HIGGS_QueryHandOperationsInterface(1, sizeof(operations), &operations))
        return 4;
    return api.SubmitCandidate && routing.CancelRecognizedAction ? 0 : 3;
}
