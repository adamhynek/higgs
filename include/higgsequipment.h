#ifndef HIGGS_EQUIPMENT_H
#define HIGGS_EQUIPMENT_H
#include "higgsauthorizeddispatch.h"
#ifdef __cplusplus
extern "C"
{
#endif
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
#pragma pack(push, 8)
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
#pragma pack(pop)
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
#ifdef __cplusplus
}
#endif
#endif
