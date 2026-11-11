#ifndef HIGGS_TRANSACTIONS_H
#define HIGGS_TRANSACTIONS_H
#include "higgshandapi.h"
#ifdef __cplusplus
extern "C"
{
#endif
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
#pragma pack(push, 8)
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
#pragma pack(pop)
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
#ifdef __cplusplus
}
#endif
#endif
