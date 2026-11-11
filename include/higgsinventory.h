#ifndef HIGGS_INVENTORY_H
#define HIGGS_INVENTORY_H
#include "higgstransactions.h"
#ifdef __cplusplus
extern "C" {
#endif
#define HIGGS_INVENTORY_VERSION 1u
#define HIGGS_INVENTORY_CAP_PLAYER_SELECTION UINT64_C(1)
#define HIGGS_INVENTORY_CAP_DROP_TO_HAND UINT64_C(2)
#define HIGGS_INVENTORY_CAP_STORE_HELD UINT64_C(4)
#define HIGGS_INVENTORY_CAP_CONSUME_HELD UINT64_C(8)
#define HIGGS_INVENTORY_CAP_GAME_EQUIPMENT UINT64_C(16)
enum { HIGGS_SELECTION_QUEUED=1, HIGGS_SELECTION_READY=2, HIGGS_SELECTION_INVALID=3 };
#pragma pack(push, 8)
typedef struct HIGGS_InventorySelectionRequest {
    uint32_t structSize, baseFormId;
    uint64_t epoch;
    uint32_t stackOrdinal, quantity;
    uint32_t reserved[2];
} HIGGS_InventorySelectionRequest;
typedef struct HIGGS_InventorySelection {
    uint32_t structSize, state;
    uint64_t ticket, epoch;
    uint32_t reason, containerRefId;
    HIGGS_ItemIdentity item;
} HIGGS_InventorySelection;
typedef struct HIGGS_InventoryInterface {
    uint32_t structSize, abiVersion;
    uint64_t capabilities;
    HIGGS_HandStatus (HIGGS_HAND_CALL *RequestSelection)(HIGGS_HandClient, const HIGGS_InventorySelectionRequest*, uint64_t*);
    HIGGS_HandStatus (HIGGS_HAND_CALL *CopySelection)(HIGGS_HandClient, uint64_t, HIGGS_InventorySelection*);
    HIGGS_HandStatus (HIGGS_HAND_CALL *ReleaseSelection)(HIGGS_HandClient, uint64_t);
    HIGGS_HandStatus (HIGGS_HAND_CALL *GetExecutor)(HIGGS_HandClient, HIGGS_HandExecutor*);
} HIGGS_InventoryInterface;
#pragma pack(pop)
/* Asynchronous player-inventory selection: ordinal zero is the unextended plain
   stack, ordinals 1..N are live extended stacks in the observed entry order.
   A selection binds the exact observed stack, not the future ordinal. No cached
   extra-data pointer is dereferenced. Selection may become stale after any relevant
   inventory change; request a new ticket, never blindly retry a mutation.
   READY supplies a HIGGS-owned native item identity for TransactionsInterface.
   GetExecutor supplies the common native executor for that same consumer client.
   Equipment uses physical destinationHand and real extra data; arbitrary containers,
   scripted activation, proxy creation and nonordinary item semantics are unsupported.
   Existing transaction grants, idempotency, cancellation, receipt provenance,
   uncertain-mutation recovery and native grip/equipment confirmation still apply.
   No game-engine pointer or extra-data address crosses this ABI. */
typedef HIGGS_HandStatus (HIGGS_HAND_CALL *HIGGS_QueryInventoryInterfaceFn)(uint32_t,uint32_t,HIGGS_InventoryInterface*);
HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryInventoryInterface(uint32_t,uint32_t,HIGGS_InventoryInterface*);
#ifdef __cplusplus
}
#endif
#endif
