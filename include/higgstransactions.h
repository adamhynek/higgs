#ifndef HIGGSTRANSACTIONS_H
#define HIGGSTRANSACTIONS_H
#include "higgshandapi.h"
#ifdef __cplusplus
extern "C" {
#endif
#pragma pack(push, 8)

#define HIGGS_TRANSACTIONS_VERSION 2u
#define HIGGS_TX_CAP_ITEM_IDENTITIES UINT64_C(1)
#define HIGGS_TX_CAP_TRANSFERS UINT64_C(2)
#define HIGGS_TX_CAP_CONSUMPTION UINT64_C(4)
#define HIGGS_TX_CAP_MUTATION_PERMITS UINT64_C(8)
#define HIGGS_TX_CAP_GAME_EQUIPMENT UINT64_C(16)
    enum
    {
        HIGGS_ITEM_DOMAIN = 1,
        HIGGS_ITEM_WORLD_REFERENCE = 2,
        HIGGS_ITEM_NATIVE_INVENTORY = 3
    };
    enum
    {
        HIGGS_ITEM_BASE_KNOWN = 1,
        HIGGS_ITEM_REFERENCE_KNOWN = 2,
        HIGGS_ITEM_QUANTITY_KNOWN = 4,
        HIGGS_ITEM_INSTANCE_ATTESTED = 8,
        HIGGS_ITEM_NATIVE_OBSERVED = 16
    };
    enum
    {
        HIGGS_TRANSFER_INVENTORY_TO_HAND = 1,
        HIGGS_TRANSFER_HAND_TO_INVENTORY = 2,
        HIGGS_TRANSFER_HAND_TO_HAND = 3,
        HIGGS_TRANSFER_CONSUME_HELD = 4,
        HIGGS_TRANSFER_EQUIP_GAME = 5,
        HIGGS_TRANSFER_UNEQUIP_GAME = 6
    };
    enum
    {
        HIGGS_TX_WAIT_RELEASE = 1,
        HIGGS_TX_WAIT_EXECUTOR = 2,
        HIGGS_TX_EXECUTING = 3,
        HIGGS_TX_WAIT_GRIP = 4,
        HIGGS_TX_WAIT_COMPENSATION = 5,
        HIGGS_TX_COMPLETED = 6,
        HIGGS_TX_REJECTED = 7,
        HIGGS_TX_CANCELLED = 8,
        HIGGS_TX_RECOVERY_REQUIRED = 9,
        HIGGS_TX_WAIT_EQUIPMENT = 10
    };
    enum
    {
        HIGGS_TX_EXECUTE = 1,
        HIGGS_TX_COMPENSATE = 2
    };
    enum
    {
        HIGGS_TX_INVENTORY_COMMITTED = 1,
        HIGGS_TX_CONSUMPTION_COMMITTED = 2,
        HIGGS_TX_COMPENSATED = 3,
        HIGGS_TX_EXECUTION_FAILED = 4,
        HIGGS_TX_RECONCILED = 5, /* actual domain-state reconciliation; no mutation permit. */
        HIGGS_TX_EQUIPMENT_COMMITTED = 6
    };
    enum
    {
        HIGGS_TX_DOMAIN_ATTESTED = 1,
        HIGGS_TX_NATIVE_RELEASE_CONFIRMED = 2,
        HIGGS_TX_NATIVE_GRIP_CONFIRMED = 4,
        HIGGS_TX_INSTANCE_PRESERVED = 8,
        HIGGS_TX_COMPENSATION_CONFIRMED = 16,
        HIGGS_TX_MUTATION_UNCERTAIN = 32,
        HIGGS_TX_RECOVERY_RECONCILED = 64,
        HIGGS_TX_NATIVE_EQUIPMENT_CONFIRMED = 128
    };
    enum
    {
        HIGGS_DISPOSITION_UNCHANGED = 0,
        HIGGS_DISPOSITION_WORLD = 1,
        HIGGS_DISPOSITION_HAND = 2,
        HIGGS_DISPOSITION_INVENTORY = 3,
        HIGGS_DISPOSITION_CONSUMED = 4,
        HIGGS_DISPOSITION_UNKNOWN = 5,
        HIGGS_DISPOSITION_GAME_EQUIPPED = 6
    };
    typedef struct HIGGS_ItemIdentity
    {
        uint32_t structSize, kind;
        uint64_t epoch, instance, generation;
        uint64_t provider; /* Public label of the declaring client. Zero for observed world items. */
        uint64_t providerKey, instanceDataVersion;
        uint32_t validFields, baseFormId, worldRefId, quantity;
    } HIGGS_ItemIdentity;
    typedef struct HIGGS_ItemDeclaration
    {
        uint32_t structSize, reserved;
        uint64_t expectedEpoch, providerKey, instanceDataVersion;
        uint32_t validFields, baseFormId, worldRefId, quantity;
    } HIGGS_ItemDeclaration;
    typedef struct HIGGS_TransferRequest
    {
        uint32_t structSize, kind, sourceHand, destinationHand;
        uint64_t epoch, idempotencyKey;
        HIGGS_ItemIdentity original;
        HIGGS_HandExecutor executor;
        HIGGS_HandGrant grant;
        uint64_t sourceResource, destinationResource, contextGeneration;
        uint32_t quantity, timeoutMs;
        uint32_t expectedEquippedFormId, requiredHands; /* equipment: base/proxy to observe; 0/auto or hand mask. */
    } HIGGS_TransferRequest;
    typedef struct HIGGS_TransferState
    {
        uint32_t structSize, stage;
        HIGGS_HandRequestId requestId;
        HIGGS_HandClient owner;
        HIGGS_HandGrant grant;
        uint64_t epoch, idempotencyKey;
        uint32_t receiptFlags, disposition, reason, physicalHands;
        HIGGS_ItemIdentity original, executionProxy, display;
        uint32_t quantity, actualRefId;
        uint32_t actualEquippedFormId, reserved;
        uint64_t mutationPermit, physicalRequestId;
    } HIGGS_TransferState;
    typedef struct HIGGS_TransferExecution
    {
        uint32_t structSize, phase;
        HIGGS_HandRequestId requestId;
        HIGGS_HandExecutor executor;
        uint64_t epoch;
        HIGGS_TransferRequest request;
        HIGGS_TransferState current;
    } HIGGS_TransferExecution;
    typedef struct HIGGS_TransferReceipt
    {
        uint32_t structSize, kind;
        HIGGS_HandRequestId requestId;
        HIGGS_HandExecutor executor;
        uint64_t epoch, mutationPermit;
        HIGGS_ItemIdentity original, executionProxy, display;
        uint32_t quantity, actualRefId, instancePreserved, reason;
        uint32_t disposition, reserved; /* only RECONCILED: world/inventory/consumed. */
        uint32_t actualEquippedFormId, reserved2;
    } HIGGS_TransferReceipt;
    typedef void(HIGGS_HAND_CALL *HIGGS_TransferExecutorCallback)(const HIGGS_TransferExecution *, void *);
    typedef struct HIGGS_TransactionsInterface
    {
        uint32_t structSize, abiVersion;
        uint64_t capabilities;
        HIGGS_HandStatus(HIGGS_HAND_CALL *DeclareItem)(HIGGS_HandClient, const HIGGS_ItemDeclaration *,
                                                       HIGGS_ItemIdentity *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyHeldItem)(HIGGS_HandClient, uint32_t, HIGGS_ItemIdentity *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *RegisterExecutor)(HIGGS_HandClient, HIGGS_TransferExecutorCallback, void *,
                                                            HIGGS_HandExecutor *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *RequestTransfer)(HIGGS_HandClient, const HIGGS_TransferRequest *,
                                                           HIGGS_HandRequestId *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *BeginMutation)(HIGGS_HandClient, HIGGS_HandRequestId, uint64_t, uint64_t *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *ReportReceipt)(HIGGS_HandClient, const HIGGS_TransferReceipt *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyTransfer)(HIGGS_HandClient, HIGGS_HandRequestId, HIGGS_TransferState *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CancelTransfer)(HIGGS_HandClient, HIGGS_HandRequestId, uint64_t);
    } HIGGS_TransactionsInterface;
    /* Additive immutable v1, existing hand clients and removable registration tokens.
       Domain declarations are explicitly provider attestations, never engine item
       observations. Native identities are minted from retained reference instances,
       not base forms; original/proxy/display identities remain distinct. No pointer
       or native extra-data address crosses the copied ABI.
       Transfers reserve named hands, item identity and owner resource keys. Repeating
       the same owner/epoch/idempotencyKey returns the same request, never dispatches
       mutation twice. Changing a request under an existing key is CONFLICT.
       Executors run only in HIGGS main maintenance. BeginMutation grants exactly one
       permit per execute/compensate phase; duplicate acquisition returns CONFLICT.
       The assigned executor owns inventory/consumption semantics and reports actual
       commitment with that permit. HIGGS independently confirms physical release /
       grip using its native hand owner, preserving receipt provenance. Hand-to-hand
       transfer releases then regrips the same retained reference without inventory.
       If grip fails after inventory commitment, the executor is called once for
       compensation. A failed/lost receipt after mutation started is RECOVERY_REQUIRED,
       never a promise of rollback or permission for a blind retry. Lifecycle resets
       invalidate live authority while preserving uncertain/committed receipt facts.
       Source/destination keys are correlation/resource locks owned by the executor,
       not slot or inventory policy. Layouts exact-version-only, sized copied records,
       callbacks use HandInterface RemoveRegistration drainage rules. */
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryTransactionsInterfaceFn)(uint32_t, uint32_t,
                                                                                  HIGGS_TransactionsInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryTransactionsInterface(uint32_t, uint32_t,
                                                                      HIGGS_TransactionsInterface *);

#define HIGGS_INVENTORY_VERSION 2u
#define HIGGS_INVENTORY_CAP_PLAYER_SELECTION UINT64_C(1)
#define HIGGS_INVENTORY_CAP_DROP_TO_HAND UINT64_C(2)
#define HIGGS_INVENTORY_CAP_STORE_HELD UINT64_C(4)
#define HIGGS_INVENTORY_CAP_CONSUME_HELD UINT64_C(8)
#define HIGGS_INVENTORY_CAP_GAME_EQUIPMENT UINT64_C(16)
enum { HIGGS_SELECTION_QUEUED=1, HIGGS_SELECTION_READY=2, HIGGS_SELECTION_INVALID=3 };
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

enum
    {
        HIGGS_EQUIP_PLACE_GAME_HAND = 1,
        HIGGS_EQUIP_PLACE_PHYSICAL_HAND = 2,
        HIGGS_EQUIP_PLACE_BIPED = 3
    };
    enum
    {
        HIGGS_EQUIP_EMPTY = 1,
        HIGGS_EQUIP_ITEM = 2
    };
    enum
    {
        HIGGS_EQUIP_ORDINARY = 1,
        HIGGS_EQUIP_MULTI_PLACE = 2,
        HIGGS_EQUIP_PROXY = 4
    };
    enum
    {
        HIGGS_EQUIP_CHECKING = 1,
        HIGGS_EQUIP_DECLINED = 2,
        HIGGS_EQUIP_AWAITING = 3,
        HIGGS_EQUIP_CONFIRMED = 4,
        HIGGS_EQUIP_RECOVERY_REQUIRED = 5,
        HIGGS_EQUIP_ORIGINAL_OBSERVED = 6
    };
    enum
    {
        HIGGS_EQUIP_BACKEND_SETTLED = 1
    };
#define HIGGS_EQUIP_MAX_PLACES 8u
    typedef struct HIGGS_EquipmentItem
    {
        uint32_t state, originalForm, equippedForm, visualForm;
        uint64_t instanceProvider, instance, generation;
    } HIGGS_EquipmentItem;
    typedef struct HIGGS_EquipmentChange
    {
        uint32_t placeKind, placeId;
        HIGGS_EquipmentItem before, after;
    } HIGGS_EquipmentChange;
    typedef struct HIGGS_EquipmentTransition
    {
        uint32_t structSize, physicalHands, placeCount, requiredCapabilities;
        HIGGS_AuthorizedDispatchRequest authority;
        HIGGS_EquipmentChange places[HIGGS_EQUIP_MAX_PLACES];
    } HIGGS_EquipmentTransition;
    typedef struct HIGGS_EquipmentEvidence
    {
        uint32_t structSize, placeCount, observedMask, flags;
        HIGGS_EquipmentItem places[HIGGS_EQUIP_MAX_PLACES];
    } HIGGS_EquipmentEvidence;
    typedef struct HIGGS_EquipmentOutcome
    {
        uint32_t structSize, stage, reason, placeCount;
        uint64_t epoch, requestId, executor;
        HIGGS_EquipmentChange expected[HIGGS_EQUIP_MAX_PLACES];
        HIGGS_EquipmentEvidence before, after;
    } HIGGS_EquipmentOutcome;
    typedef struct HIGGS_EquipmentBackend
    {
        uint32_t structSize, capabilities;
        HIGGS_HandStatus(HIGGS_HAND_CALL *Observe)(const HIGGS_EquipmentTransition *, HIGGS_EquipmentEvidence *,
                                                   void *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *Apply)(const HIGGS_EquipmentTransition *, void *);
        void *context;
    } HIGGS_EquipmentBackend;
    typedef struct HIGGS_EquipmentInterface
    {
        uint32_t structSize, abiVersion;
        uint64_t nativeLayoutCapabilities; /* Exact complete-layout backends integrated into this table. */
        HIGGS_HandStatus(HIGGS_HAND_CALL *Request)(HIGGS_HandClient, const HIGGS_EquipmentTransition *,
                                                   const HIGGS_EquipmentBackend *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyOutcome)(HIGGS_HandClient, uint64_t epoch, uint64_t requestId,
                                                       HIGGS_EquipmentOutcome *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *ReportObservation)(HIGGS_HandClient, uint64_t epoch, uint64_t requestId,
                                                             const HIGGS_EquipmentEvidence *);
    } HIGGS_EquipmentInterface;
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryEquipmentInterface(uint32_t, uint32_t, HIGGS_EquipmentInterface *);
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryEquipmentInterfaceFn)(uint32_t, uint32_t,
                                                                               HIGGS_EquipmentInterface *);
/* The already selected external executor calls Request at main maintenance instead
   of Invoke, with its own synchronous backend. No backend pointer is retained.
   Observe is read-only and fills one position per requested place, in order.
   Only Apply mutates. Request's OK means the outcome can be read. Before facts
   must all match. After confirmation requires all touched places and exact
   executor-attested item instances. Index 1/2 means main/off for GAME_HAND and
   physical left/right for PHYSICAL_HAND. Biped indices are 30..61. Place IDs,
   GAME_HAND and BIPED require both physical hands because the backend owns the projection.
   instanceProvider is a public identity namespace, never a client credential.
   Exact instance keys and individual capability bits are independent of form type.
   A conditional backend declines unsupported layouts in the first Observe call.
   CopyOutcome contains executor evidence, never a native wield/block guarantee.
   On a later frame the backend may report more evidence. BACKEND_SETTLED means the
   backend certifies it has finished and cannot apply more delayed changes. An
   intermediate before or after layout never terminates a pending transition. A caller that
   receives a partial or uncertain outcome must reconcile instead of reapplying.
   Ordinary native equip remains available through the existing transfer interface. */

#pragma pack(pop)
#ifdef __cplusplus
}
#endif
#endif
