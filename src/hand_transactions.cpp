#include "hand_transactions.h"
#include "hand_broker.h"
#include <algorithm>
#include <cstring>

namespace HiggsHands
{
namespace
{
bool TerminalTx(uint32_t stage)
{
    return stage >= HIGGS_TX_COMPLETED && stage <= HIGGS_TX_RECOVERY_REQUIRED;
}
bool PhysicalHand(uint32_t hand)
{
    return hand == HIGGS_HAND_LEFT || hand == HIGGS_HAND_RIGHT;
}
bool EqualIdentity(const HIGGS_ItemIdentity &a, const HIGGS_ItemIdentity &b)
{
    return a.epoch == b.epoch && a.instance == b.instance && a.generation == b.generation && a.provider == b.provider && a.providerKey == b.providerKey &&
           a.instanceDataVersion == b.instanceDataVersion && a.quantity == b.quantity && a.baseFormId == b.baseFormId &&
           a.worldRefId == b.worldRefId && a.kind == b.kind && a.validFields == b.validFields;
}
} // namespace
bool Broker::ClientValid(uint64_t owner)
{
    std::lock_guard lock(mutex_);
    return ClientLocked(owner) != nullptr;
}
bool Broker::ExecutorValid(uint64_t owner, uint64_t token, uint32_t kind)
{
    std::lock_guard lock(mutex_);
    auto r = RegistrationLocked(token);
    return ClientLocked(owner) && r && r->active && r->owner == owner && r->kind == kind;
}
HIGGS_HandStatus Broker::TransferExecutor(uint64_t owner, HIGGS_TransferExecutorCallback cb, void *context,
                                          uint64_t &out)
{
    if (!cb)
        return HIGGS_HAND_INVALID_ARGUMENT;
    auto status = Add(owner, 4, nullptr, nullptr, nullptr, context, out);
    if (!status)
    {
        std::lock_guard lock(mutex_);
        RegistrationLocked(out)->transfer = cb;
    }
    return status;
}
HIGGS_HandStatus Broker::ReserveDomain(uint64_t owner, uint64_t executor, uint32_t hands, uint64_t epoch,
                                       uint64_t grant, uint32_t timeout, HIGGS_ActionRequest &out, uint64_t now)
{
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    if (!grant && SharedRoundsLocked()) return HIGGS_HAND_WRONG_PHASE;
    if (worldChangePending_ || contextMenus_)
        return HIGGS_HAND_NOT_READY;
    if (epoch != snapshot_.epoch)
        return HIGGS_HAND_STALE;
    if (!hands || (hands & ~3u) || !timeout || timeout > 60000)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (executor)
    {
        auto r = RegistrationLocked(executor);
        if (!r || !r->active || r->kind != 4 || r->owner != owner)
            return HIGGS_HAND_WRONG_OWNER;
    }
    uint64_t deadline = now + static_cast<uint64_t>(timeout) * 1000000, input = 0;
    for (const auto &h : snapshot_.hands)
        if (h.physicalHand & hands)
        {
            if (h.availability == HIGGS_AVAILABILITY_UNKNOWN || h.availability == HIGGS_AVAILABILITY_UNAVAILABLE ||
                (h.observationMask & HIGGS_OBS_SECONDARY_SUPPORT))
                return HIGGS_HAND_NOT_READY;
            if (grant)
            {
                if (h.grant != grant || h.owner != owner)
                    return HIGGS_HAND_WRONG_OWNER;
            }
            else if (h.grant)
                return HIGGS_HAND_CONFLICT;
        }
    if (grant)
    {
        if (!GrantReadyLocked(owner, grant))
            return HIGGS_HAND_WRONG_PHASE;
        bool found = false;
        for (const auto &a : actions_)
            if (a.grantActive && a.request.grant == grant && a.request.owner == owner)
            {
                deadline = (std::min)(deadline, a.expires);
                input = a.request.inputId;
                found = true;
                break;
            }
        if (!found)
            return HIGGS_HAND_STALE;
    }
    auto a = NewActionLocked();
    if (!a)
        return HIGGS_HAND_CAPACITY;
    a->request.structSize = sizeof(a->request);
    a->request.requestId = nextToken_++;
    a->request.owner = owner;
    a->request.executor = executor;
    a->request.physicalHands = hands;
    a->request.epoch = epoch;
    a->request.inputId = input;
    a->request.grant = grant ? grant : nextToken_++;
    a->grantActive = !grant;
    a->selected = true;
    a->domainOwned = true;
    a->retained = true;
    a->expires = deadline;
    FinishLocked(*a, HIGGS_ACTION_ACCEPTED, 0);
    RefreshLocked();
    out = a->request;
    return HIGGS_HAND_OK;
}
bool Broker::DomainAlive(uint64_t owner, uint64_t request, uint64_t now)
{
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    auto a = ActionLocked(request);
    if (!ClientLocked(owner) || !a || a->request.owner != owner || a->request.epoch != snapshot_.epoch ||
        (now>=a->expires&&!a->nativeStarted) ||
        a->result.stage == HIGGS_ACTION_CANCELLED || a->result.stage == HIGGS_ACTION_REJECTED)
        return false;
    for (const auto &h : snapshot_.hands)
        if (h.physicalHand & a->request.physicalHands)
            if (h.grant != a->request.grant || h.owner != owner)
                return false;
    return true;
}
HIGGS_HandStatus Broker::ClaimActuator(uint64_t request, uint32_t kind)
{
    std::lock_guard lock(mutex_);
    auto action = ActionLocked(request);
    if (!action || !action->domainOwned || !action->retained || (kind != 1 && kind != 2))
        return HIGGS_HAND_INVALID_ARGUMENT;
    for (const auto &other : actions_)
    {
        if (other.request.requestId == request || !(other.request.physicalHands & action->request.physicalHands))
            continue;
        if (other.ownedActuator)
            return HIGGS_HAND_CONFLICT;
        if (kind == 1 && other.operationActive &&
            (other.operation.operation.kind == HIGGS_OP_WORLD_SUPPORT ||
             other.operation.operation.kind == HIGGS_OP_PHYSICAL_HAND_TARGET))
            return HIGGS_HAND_CONFLICT;
    }
    action->ownedActuator = kind;
    return HIGGS_HAND_OK;
}
void Broker::UnclaimActuator(uint64_t request)
{
    std::lock_guard lock(mutex_);
    if (auto action = ActionLocked(request))
        action->ownedActuator = 0;
}
bool Broker::EnterDomainMutation(uint64_t request, bool cleanup)
{
    std::lock_guard lock(mutex_);
    ExpireLocked(Now());
    auto action = ActionLocked(request);
    if (!action || !action->retained || !action->domainOwned || action->nativeStarted)
        return false;
    if (!cleanup)
    {
        if (contextMenus_ && action->result.stage < HIGGS_ACTION_COMMITTED)
            return false;
        if (worldChangePending_ || action->request.epoch != snapshot_.epoch || !ClientLocked(action->request.owner))
            return false;
        for (const auto &hand : snapshot_.hands)
            if (hand.physicalHand & action->request.physicalHands)
                if (hand.grant != action->request.grant || hand.owner != action->request.owner)
                    return false;
    }
    action->nativeStarted = true;
    return true;
}
void Broker::ExitDomainMutation(uint64_t request)
{
    {
        std::lock_guard lock(mutex_);
        if (auto action = ActionLocked(request))
            action->nativeStarted = false;
    }
    ApplyDeferredWorld();
}
void Broker::DomainResult(uint64_t request, uint32_t stage, uint32_t flags, uint32_t ref, uint32_t quantity)
{
    std::lock_guard lock(mutex_);
    auto a = ActionLocked(request);
    if (!a || !a->domainOwned)
        return;
    a->result.receiptFlags |= flags;
    a->result.itemRefId = ref;
    a->result.quantity = quantity;
    if (a->result.stage >= HIGGS_ACTION_COMPLETED)
        return;
    FinishLocked(*a, stage, 0);
}
void Broker::RetainRequest(uint64_t request, bool retain)
{
    std::lock_guard lock(mutex_);
    if (auto a = ActionLocked(request))
        a->retained = retain;
}
HIGGS_HandStatus Broker::CancelNativeRequest(uint64_t owner, uint64_t request)
{
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    auto a = ActionLocked(request);
    if (!a)
        return HIGGS_HAND_NOT_FOUND;
    if (a->request.owner != owner)
        return HIGGS_HAND_WRONG_OWNER;
    if (a->nativeStarted)
        return HIGGS_HAND_PENDING;
    if (a->result.stage < HIGGS_ACTION_COMPLETED)
    {
        a->grantActive = false;
        a->operationActive = false;
        FinishLocked(*a, HIGGS_ACTION_CANCELLED, 0);
        RefreshLocked();
    }
    return HIGGS_HAND_OK;
}
TransactionService &Transactions()
{
    static TransactionService service(Service());
    return service;
}
TransactionService::Transaction *TransactionService::Find(uint64_t id)
{
    for (auto &t : transactions_)
        if (id && t.state.requestId == id)
            return &t;
    return nullptr;
}
TransactionService::Item *TransactionService::FindItem(uint64_t id)
{
    for (auto &item : items_)
        if (id && item.value.instance == id)
            return &item;
    return nullptr;
}
bool TransactionService::LockedItem(uint64_t item) const
{
    for (const auto &t : transactions_)
        if (t.state.requestId && t.state.epoch == epoch_ &&
            (!TerminalTx(t.state.stage) || t.state.stage == HIGGS_TX_RECOVERY_REQUIRED) &&
            t.request.original.instance == item)
            return true;
    return false;
}
bool TransactionService::IdentityLocked(const HIGGS_ItemIdentity &id) const
{
    for (const auto &item : items_)
        if (item.value.instance == id.instance && EqualIdentity(item.value, id))
            return true;
    return false;
}
void TransactionService::Finish(Transaction &t, uint32_t stage, uint32_t disposition, uint32_t reason)
{
    t.state.stage = stage;
    t.state.disposition = disposition;
    t.state.reason = reason;
    broker_.RetainRequest(t.state.physicalRequestId, false);
    broker_.RetainRequest(t.state.requestId, false);
    if (stage == HIGGS_TX_COMPLETED)
        broker_.DomainResult(
            t.state.requestId, HIGGS_ACTION_COMPLETED,
            ((t.state.receiptFlags & HIGGS_TX_DOMAIN_ATTESTED) && t.request.kind != HIGGS_TRANSFER_EQUIP_GAME &&
                     t.request.kind != HIGGS_TRANSFER_UNEQUIP_GAME
                 ? HIGGS_RECEIPT_INVENTORY_COMMITTED
                 : 0u) |
                ((t.state.receiptFlags & (HIGGS_TX_NATIVE_GRIP_CONFIRMED | HIGGS_TX_NATIVE_RELEASE_CONFIRMED |
                                          HIGGS_TX_NATIVE_EQUIPMENT_CONFIRMED))
                     ? HIGGS_RECEIPT_PHYSICAL_CONFIRMED
                     : 0u),
            t.state.actualRefId, t.state.quantity);
    else
        broker_.DomainResult(t.state.requestId, HIGGS_ACTION_CANCELLED, 0, t.state.actualRefId, t.state.quantity);
    if (t.ownGrant)
        broker_.Release(t.state.owner, t.state.grant);
}
void TransactionService::SyncLocked()
{
    const auto epoch = broker_.Epoch();
    if (epoch == epoch_)
        return;
    for (auto &t : transactions_)
        if (t.state.requestId && !TerminalTx(t.state.stage))
        {
            const bool uncertain = t.permitIssued || t.state.receiptFlags & HIGGS_TX_DOMAIN_ATTESTED;
            if (uncertain)
                t.state.receiptFlags |= HIGGS_TX_MUTATION_UNCERTAIN;
            Finish(t, uncertain ? HIGGS_TX_RECOVERY_REQUIRED : HIGGS_TX_CANCELLED,
                   uncertain ? HIGGS_DISPOSITION_UNKNOWN : HIGGS_DISPOSITION_UNCHANGED, HIGGS_HAND_STALE);
        }
    items_ = {};
    keys_.clear();
    held_[0] = held_[1] = 0;
    epoch_ = epoch;
}
HIGGS_HandStatus TransactionService::Declare(uint64_t owner, const HIGGS_ItemDeclaration &d, HIGGS_ItemIdentity &out)
{
    auto phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    SyncLocked();
    if (!broker_.ClientValid(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    if (d.structSize < sizeof(d) || d.reserved || !d.providerKey || !d.instanceDataVersion || !d.quantity ||
        (d.validFields & ~(HIGGS_ITEM_BASE_KNOWN | HIGGS_ITEM_REFERENCE_KNOWN | HIGGS_ITEM_QUANTITY_KNOWN |
                           HIGGS_ITEM_INSTANCE_ATTESTED)) ||
        !(d.validFields & HIGGS_ITEM_INSTANCE_ATTESTED) || !(d.validFields & HIGGS_ITEM_QUANTITY_KNOWN))
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (d.expectedEpoch != epoch_)
        return HIGGS_HAND_STALE;
    Item *slot = nullptr;
    for (auto &item : items_)
        if (item.value.kind == HIGGS_ITEM_DOMAIN && item.owner == owner &&
            item.value.providerKey == d.providerKey)
        {
            slot = &item;
            break;
        }
    if (slot && LockedItem(slot->value.instance))
    {
        const auto &v = slot->value;
        if (v.instanceDataVersion != d.instanceDataVersion || v.quantity != d.quantity ||
            v.baseFormId != d.baseFormId || v.worldRefId != d.worldRefId || v.validFields != d.validFields)
            return HIGGS_HAND_CONFLICT;
        out = v;
        return HIGGS_HAND_OK;
    }
    if (!slot)
        for (auto &item : items_)
            if (!item.value.instance)
            {
                slot = &item;
                break;
            }
    if (!slot)
        return HIGGS_HAND_CAPACITY;
    auto &v = slot->value;
    if (v.instance && d.instanceDataVersion < v.instanceDataVersion)
        return HIGGS_HAND_STALE;
    if (v.instance && d.instanceDataVersion == v.instanceDataVersion &&
        (v.quantity != d.quantity || v.baseFormId != d.baseFormId || v.worldRefId != d.worldRefId ||
         v.validFields != d.validFields))
        return HIGGS_HAND_CONFLICT;
    if (!v.instance || d.instanceDataVersion > v.instanceDataVersion)
        slot->available = true;
    if (!v.instance)
    {
        v.instance = next_++;
        v.generation = 1;
    }
    else if (v.instanceDataVersion != d.instanceDataVersion || v.quantity != d.quantity ||
             v.baseFormId != d.baseFormId || v.worldRefId != d.worldRefId || v.validFields != d.validFields)
        ++v.generation;
    v.structSize = sizeof(v);
    v.kind = HIGGS_ITEM_DOMAIN;
    v.epoch = epoch_;
    slot->owner = owner;
    v.provider = broker_.ClientLabel(owner);
    v.providerKey = d.providerKey;
    v.instanceDataVersion = d.instanceDataVersion;
    v.quantity = d.quantity;
    v.baseFormId = d.baseFormId;
    v.worldRefId = d.worldRefId;
    v.validFields = d.validFields;
    out = v;
    return HIGGS_HAND_OK;
}
uint64_t TransactionService::ObserveHeld(uint32_t hand, uint64_t nativeKey, uint32_t ref, uint32_t base, uint32_t count,
                                         uint64_t now)
{
    std::lock_guard lock(mutex_);
    SyncLocked();
    if (!PhysicalHand(hand))
        return 0;
    const auto hi = hand == HIGGS_HAND_LEFT ? 0 : 1;
    if (!nativeKey || !ref || !count)
    {
        held_[hi] = 0;
        return 0;
    }
    Item *slot = nullptr;
    for (auto &item : items_)
        if (item.nativeKey == nativeKey && item.value.epoch == epoch_ && item.value.kind == HIGGS_ITEM_WORLD_REFERENCE)
        {
            slot = &item;
            break;
        }
    if (!slot)
        for (auto &item : items_)
            if (!item.value.instance)
            {
                slot = &item;
                break;
            }
    if (!slot)
        return 0;
    auto &v = slot->value;
    if (!v.instance)
    {
        v.instance = next_++;
        v.generation = 1;
    }
    else if (v.worldRefId != ref || v.quantity != count || v.baseFormId != base)
        ++v.generation;
    v.structSize = sizeof(v);
    v.kind = HIGGS_ITEM_WORLD_REFERENCE;
    v.epoch = epoch_;
    v.provider = 0;
    v.providerKey = 0;
    v.instanceDataVersion = v.generation;
    v.worldRefId = ref;
    v.baseFormId = base;
    v.quantity = count;
    v.validFields = HIGGS_ITEM_REFERENCE_KNOWN | HIGGS_ITEM_QUANTITY_KNOWN | HIGGS_ITEM_NATIVE_OBSERVED |
                    (base ? HIGGS_ITEM_BASE_KNOWN : 0u);
    slot->nativeKey = nativeKey;
    slot->seen = now;
    held_[hi] = v.instance;
    return v.instance;
}
HIGGS_HandStatus TransactionService::ObserveInventory(uint64_t owner, uint64_t key, uint64_t version, uint32_t base,
                                                      uint32_t quantity, HIGGS_ItemIdentity &out)
{
    auto phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    SyncLocked();
    if (!broker_.ClientValid(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    if (!key || !version || !base || !quantity)
        return HIGGS_HAND_INVALID_ARGUMENT;
    Item *slot = nullptr;
    for (auto &item : items_)
        if (item.value.kind == HIGGS_ITEM_NATIVE_INVENTORY && item.owner == owner &&
            item.value.providerKey == key)
        {
            slot = &item;
            break;
        }
    if (slot && LockedItem(slot->value.instance))
    {
        const auto &value = slot->value;
        if (!slot->available || value.instanceDataVersion != version || value.baseFormId != base ||
            value.quantity != quantity)
            return HIGGS_HAND_CONFLICT;
        out = value;
        return HIGGS_HAND_OK;
    }
    if (!slot)
        for (auto &item : items_)
            if (!item.value.instance)
            {
                slot = &item;
                break;
            }
    if (!slot)
        return HIGGS_HAND_CAPACITY;
    auto &value = slot->value;
    if (value.instance && version < value.instanceDataVersion)
        return HIGGS_HAND_STALE;
    if (value.instance && version == value.instanceDataVersion &&
        (value.baseFormId != base || value.quantity != quantity))
        return HIGGS_HAND_CONFLICT;
    if (!value.instance)
    {
        value.instance = next_++;
        value.generation = 1;
        slot->available = true;
    }
    else if (version > value.instanceDataVersion)
    {
        ++value.generation;
        slot->available = true;
    }
    value.structSize = sizeof(value);
    value.kind = HIGGS_ITEM_NATIVE_INVENTORY;
    value.epoch = epoch_;
    slot->owner = owner;
    value.provider = broker_.ClientLabel(owner);
    value.providerKey = key;
    value.instanceDataVersion = version;
    value.baseFormId = base;
    value.worldRefId = 0;
    value.quantity = quantity;
    value.validFields = HIGGS_ITEM_BASE_KNOWN | HIGGS_ITEM_QUANTITY_KNOWN | HIGGS_ITEM_NATIVE_OBSERVED;
    out = value;
    return HIGGS_HAND_OK;
}
void TransactionService::InvalidateInventory(uint64_t owner, uint64_t key)
{
    auto phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    SyncLocked();
    for (auto &item : items_)
        if (item.value.kind == HIGGS_ITEM_NATIVE_INVENTORY && item.owner == owner &&
            item.value.providerKey == key)
            item.available = false;
}
bool TransactionService::RetireInventory(uint64_t owner,uint64_t key)
{
    auto phase=broker_.NativePhase();
    std::lock_guard lock(mutex_);
    SyncLocked();
    for(auto& item:items_)
        if(item.value.kind==HIGGS_ITEM_NATIVE_INVENTORY&&item.owner==owner&&item.value.providerKey==key){
            if(LockedItem(item.value.instance))return false;
            item={};
            return true;
        }
    return true;
}
bool TransactionService::RetireNativeItem(uint64_t instance)
{
    auto phase=broker_.NativePhase();
    std::lock_guard lock(mutex_);
    SyncLocked();
    if(held_[0]==instance||held_[1]==instance||LockedItem(instance))return false;
    for(auto& item:items_)
        if(item.value.instance==instance&&item.value.kind==HIGGS_ITEM_WORLD_REFERENCE){item={};return true;}
    return true;
}
bool TransactionService::NativePinRequired(uint64_t owner,uint64_t request,uint64_t epoch,HIGGS_TransferState& out)
{
    auto phase=broker_.NativePhase();
    std::lock_guard lock(mutex_);
    SyncLocked();
    const auto t=Find(request);
    if(!t||t->state.owner!=owner||t->state.epoch!=epoch||epoch!=epoch_)return false;
    out=t->state;
    return !TerminalTx(out.stage)||out.stage==HIGGS_TX_RECOVERY_REQUIRED;
}
bool TransactionService::ItemInUse(uint64_t id)
{
    std::lock_guard lock(mutex_);
    return LockedItem(id);
}
bool TransactionService::IdentityCurrent(const HIGGS_ItemIdentity &identity)
{
    std::lock_guard lock(mutex_);
    SyncLocked();
    return identity.epoch == epoch_ && IdentityLocked(identity);
}
HIGGS_HandStatus TransactionService::Held(uint64_t owner, uint32_t hand, HIGGS_ItemIdentity &out)
{
    auto phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    SyncLocked();
    if (!broker_.ClientValid(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    if (!PhysicalHand(hand))
        return HIGGS_HAND_INVALID_ARGUMENT;
    auto item = FindItem(held_[hand == HIGGS_HAND_LEFT ? 0 : 1]);
    if (!item)
        return HIGGS_HAND_NOT_FOUND;
    HIGGS_HandSnapshot s{};
    auto status = broker_.Copy(owner, s, Now());
    if (status)
        return status;
    const auto &h = s.hands[hand == HIGGS_HAND_LEFT ? 0 : 1];
    if (!(h.observationMask & HIGGS_OBS_HELD_KNOWN) || h.heldRefId != item->value.worldRefId)
        return HIGGS_HAND_STALE;
    out = item->value;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus TransactionService::Request(uint64_t owner, const HIGGS_TransferRequest &r, uint64_t &out,
                                             uint64_t now)
{
    auto phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    SyncLocked();
    if (!broker_.ClientValid(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    if (r.structSize < sizeof(r) || r.kind < HIGGS_TRANSFER_INVENTORY_TO_HAND || r.kind > HIGGS_TRANSFER_UNEQUIP_GAME ||
        !r.idempotencyKey || !r.quantity || !r.timeoutMs || r.timeoutMs > 60000 ||
        r.original.structSize < sizeof(r.original))
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (r.epoch != epoch_)
        return HIGGS_HAND_STALE;
    for (const auto &key : keys_)
        if (key.owner == owner && key.epoch == r.epoch && key.input.idempotencyKey == r.idempotencyKey)
        {
            if (std::memcmp(&key.input, &r, sizeof(r)))
                return HIGGS_HAND_CONFLICT;
            out = key.request;
            return HIGGS_HAND_OK;
        }
    if (keys_.size() >= 4096)
        return HIGGS_HAND_CAPACITY;
    for (auto &t : transactions_)
        if (t.state.owner == owner && t.state.epoch == r.epoch && t.state.idempotencyKey == r.idempotencyKey)
        {
            if (std::memcmp(&t.request, &r, sizeof(r)))
                return HIGGS_HAND_CONFLICT;
            out = t.state.requestId;
            return HIGGS_HAND_OK;
        }
    if (!IdentityLocked(r.original) || r.original.epoch != epoch_)
        return HIGGS_HAND_STALE;
    if (r.quantity != r.original.quantity)
        return HIGGS_HAND_INVALID_ARGUMENT;
    const bool equipment = r.kind == HIGGS_TRANSFER_EQUIP_GAME || r.kind == HIGGS_TRANSFER_UNEQUIP_GAME;
    const bool inventoryIn = r.kind == HIGGS_TRANSFER_INVENTORY_TO_HAND || equipment,
               physicalOnly = r.kind == HIGGS_TRANSFER_HAND_TO_HAND;
    if ((inventoryIn && !PhysicalHand(r.destinationHand)) || (!inventoryIn && !PhysicalHand(r.sourceHand)) ||
        (physicalOnly && (!PhysicalHand(r.destinationHand) || r.sourceHand == r.destinationHand)))
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (!physicalOnly && !broker_.ExecutorValid(owner, r.executor, 4))
        return HIGGS_HAND_WRONG_OWNER;
    if (inventoryIn && (r.original.provider != broker_.ClientLabel(owner) ||
                        (r.original.kind != HIGGS_ITEM_DOMAIN && r.original.kind != HIGGS_ITEM_NATIVE_INVENTORY)))
        return HIGGS_HAND_WRONG_OWNER;
    if (inventoryIn)
    {
        auto item = FindItem(r.original.instance);
        if (!item || !item->available)
            return HIGGS_HAND_CONFLICT;
    }
    HIGGS_HandSnapshot s{};
    auto status = broker_.Copy(owner, s, now);
    if (status)
        return status;
    if (inventoryIn || physicalOnly)
    {
        const auto &h = s.hands[r.destinationHand == HIGGS_HAND_LEFT ? 0 : 1];
        if (r.kind == HIGGS_TRANSFER_UNEQUIP_GAME)
        {
            if (!(h.observationMask & HIGGS_OBS_EQUIPMENT_KNOWN) || h.heldRefId || !h.equippedFormId ||
                h.equippedFormId != r.expectedEquippedFormId)
                return HIGGS_HAND_CONFLICT;
        }
        else if (h.occupancy != HIGGS_OCCUPANCY_EMPTY)
            return HIGGS_HAND_CONFLICT;
    }
    if (!inventoryIn)
    {
        auto item = FindItem(held_[r.sourceHand == HIGGS_HAND_LEFT ? 0 : 1]);
        if (!item || !EqualIdentity(item->value, r.original))
            return HIGGS_HAND_STALE;
    }
    for (const auto &t : transactions_)
        if (t.state.requestId && t.state.epoch == epoch_ &&
            (!TerminalTx(t.state.stage) || t.state.stage == HIGGS_TX_RECOVERY_REQUIRED))
        {
            if (t.request.original.instance == r.original.instance)
                return HIGGS_HAND_CONFLICT;
            if (t.state.owner == owner &&
                ((r.sourceResource && (r.sourceResource == t.request.sourceResource ||
                                       r.sourceResource == t.request.destinationResource)) ||
                 (r.destinationResource && (r.destinationResource == t.request.sourceResource ||
                                            r.destinationResource == t.request.destinationResource))))
                return HIGGS_HAND_CONFLICT;
        }
    Transaction *slot = nullptr;
    for (auto &t : transactions_)
        if (!t.state.requestId)
        {
            slot = &t;
            break;
        }
    if (!slot)
        for (auto &t : transactions_)
            if (TerminalTx(t.state.stage) && (t.state.stage != HIGGS_TX_RECOVERY_REQUIRED || t.state.epoch != epoch_) &&
                (!slot || t.state.requestId < slot->state.requestId))
                slot = &t;
    if (!slot)
        return HIGGS_HAND_CAPACITY;
    keys_.push_back(Key{owner, r.epoch, 0, r});
    const uint32_t minimumHands =
        (inventoryIn ? 0u : r.sourceHand) | ((inventoryIn || physicalOnly) ? r.destinationHand : 0u);
    const uint32_t hands = r.requiredHands ? r.requiredHands : minimumHands;
    if ((hands & ~3u) || (hands & minimumHands) != minimumHands)
    {
        keys_.pop_back();
        return HIGGS_HAND_INVALID_ARGUMENT;
    }
    if (equipment && !r.expectedEquippedFormId)
    {
        keys_.pop_back();
        return HIGGS_HAND_INVALID_ARGUMENT;
    }
    HIGGS_ActionRequest authority{};
    status = broker_.ReserveDomain(owner, physicalOnly ? 0 : r.executor, hands, r.epoch, r.grant, r.timeoutMs,
                                   authority, now);
    if (status)
    {
        keys_.pop_back();
        return status;
    }
    *slot = {};
    slot->authority = authority;
    slot->request = r;
    slot->ownGrant = !r.grant;
    slot->deadline = now + static_cast<uint64_t>(r.timeoutMs) * 1000000;
    auto &state = slot->state;
    state.structSize = sizeof(state);
    state.requestId = authority.requestId;
    state.owner = owner;
    state.grant = authority.grant;
    state.epoch = r.epoch;
    state.idempotencyKey = r.idempotencyKey;
    state.original = r.original;
    state.quantity = r.quantity;
    state.physicalHands = hands;
    state.stage = inventoryIn ? HIGGS_TX_WAIT_EXECUTOR : HIGGS_TX_WAIT_RELEASE;
    state.disposition = HIGGS_DISPOSITION_UNCHANGED;
    out = state.requestId;
    keys_.back().request = state.requestId;
    return HIGGS_HAND_OK;
}
bool TransactionService::Alive(Transaction &t, uint64_t now)
{
    return broker_.DomainAlive(t.state.owner, t.state.requestId, now);
}
void TransactionService::NeedCompensation(Transaction &t, uint32_t reason, uint64_t now)
{
    broker_.CancelNativeRequest(t.state.owner, t.state.physicalRequestId);
    broker_.RetainRequest(t.state.physicalRequestId, false);
    t.state.physicalRequestId = 0;
    t.state.stage = HIGGS_TX_WAIT_COMPENSATION;
    t.state.reason = reason;
    t.state.disposition = HIGGS_DISPOSITION_WORLD;
    t.notified = false;
    t.permitIssued = false;
    t.state.mutationPermit = 0;
    t.deadline = now + static_cast<uint64_t>(t.request.timeoutMs) * 1000000;
}
HIGGS_HandStatus TransactionService::Begin(uint64_t owner, uint64_t id, uint64_t epoch, uint64_t &permit)
{
    auto phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    SyncLocked();
    if (!broker_.ClientValid(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    auto t = Find(id);
    if (!t)
        return HIGGS_HAND_NOT_FOUND;
    if (t->state.owner != owner)
        return HIGGS_HAND_WRONG_OWNER;
    if (epoch != epoch_ || epoch != t->state.epoch)
        return HIGGS_HAND_STALE;
    if (!t->notified || (t->state.stage != HIGGS_TX_EXECUTING && t->state.stage != HIGGS_TX_WAIT_COMPENSATION))
        return HIGGS_HAND_WRONG_PHASE;
    if (!broker_.MutationAdmissionReady(t->state.stage == HIGGS_TX_WAIT_COMPENSATION))
        return HIGGS_HAND_NOT_READY;
    if (!broker_.InTransferCallback(owner, t->request.executor))
        return HIGGS_HAND_WRONG_PHASE;
    if (t->permitIssued)
        return HIGGS_HAND_CONFLICT;
    if (t->state.stage == HIGGS_TX_EXECUTING && !Alive(*t, Now()))
        return HIGGS_HAND_STALE;
    t->permitIssued = true;
    t->compensationPermit = t->state.stage == HIGGS_TX_WAIT_COMPENSATION;
    t->state.mutationPermit = next_++;
    permit = t->state.mutationPermit;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus TransactionService::Report(uint64_t owner, const HIGGS_TransferReceipt &r)
{
    auto phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    SyncLocked();
    if (!broker_.ClientValid(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    auto t = Find(r.requestId);
    if (!t)
        return HIGGS_HAND_NOT_FOUND;
    if (t->state.owner != owner || t->request.executor != r.executor)
        return HIGGS_HAND_WRONG_OWNER;
    if (r.structSize < sizeof(r) || r.kind < HIGGS_TX_INVENTORY_COMMITTED || r.kind > HIGGS_TX_EQUIPMENT_COMMITTED ||
        r.reserved || r.reserved2)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (r.epoch != epoch_ || r.epoch != t->state.epoch)
        return HIGGS_HAND_STALE;
    if ((t->lastReceipt.structSize && std::memcmp(&t->lastReceipt, &r, sizeof(r)) == 0) ||
        (t->compensationReceipt.structSize && std::memcmp(&t->compensationReceipt, &r, sizeof(r)) == 0))
        return HIGGS_HAND_OK;
    if (r.kind == HIGGS_TX_RECONCILED)
    {
        if (t->state.stage != HIGGS_TX_RECOVERY_REQUIRED || r.mutationPermit ||
            !EqualIdentity(r.original, t->state.original) || r.quantity != t->state.quantity || !r.instancePreserved ||
            (r.disposition != HIGGS_DISPOSITION_WORLD && r.disposition != HIGGS_DISPOSITION_INVENTORY &&
             r.disposition != HIGGS_DISPOSITION_CONSUMED) ||
            !broker_.ExecutorValid(owner, r.executor, 4))
            return HIGGS_HAND_CONFLICT;
        if (r.disposition == HIGGS_DISPOSITION_WORLD && !r.actualRefId)
            return HIGGS_HAND_INVALID_ARGUMENT;
        if (auto item = FindItem(t->state.original.instance))
            item->available = r.disposition == HIGGS_DISPOSITION_INVENTORY;
        t->compensationReceipt = r;
        t->state.receiptFlags |= HIGGS_TX_DOMAIN_ATTESTED | HIGGS_TX_RECOVERY_RECONCILED;
        t->state.actualRefId = r.actualRefId;
        Finish(*t, HIGGS_TX_COMPLETED, r.disposition, r.reason);
        return HIGGS_HAND_OK;
    }
    if (!t->permitIssued || r.mutationPermit != t->state.mutationPermit ||
        !EqualIdentity(r.original, t->state.original) || r.quantity != t->state.quantity)
        return HIGGS_HAND_CONFLICT;
    if (!broker_.ExecutorValid(owner, r.executor, 4))
        return HIGGS_HAND_WRONG_OWNER;
    if (r.kind == HIGGS_TX_COMPENSATED)
    {
        if (t->state.stage != HIGGS_TX_WAIT_COMPENSATION || !t->compensationPermit)
            return HIGGS_HAND_WRONG_PHASE;
        const bool equipment =
            t->request.kind == HIGGS_TRANSFER_EQUIP_GAME || t->request.kind == HIGGS_TRANSFER_UNEQUIP_GAME;
        const uint32_t restored =
            t->request.kind == HIGGS_TRANSFER_UNEQUIP_GAME ? t->request.expectedEquippedFormId : 0;
        if (equipment && (!r.instancePreserved || r.actualEquippedFormId != restored))
            return HIGGS_HAND_CONFLICT;
        t->compensationReceipt = r;
        if (auto item = FindItem(t->state.original.instance))
            item->available = true;
        t->state.receiptFlags |= HIGGS_TX_COMPENSATION_CONFIRMED | HIGGS_TX_DOMAIN_ATTESTED;
        if (equipment)
        {
            t->state.actualEquippedFormId = restored;
            t->equipmentObservation = broker_.CopyPublished().observationId;
            t->state.stage = HIGGS_TX_WAIT_EQUIPMENT;
            t->deadline = Now() + UINT64_C(2000000000);
            return HIGGS_HAND_OK;
        }
        Finish(*t, HIGGS_TX_COMPLETED, HIGGS_DISPOSITION_INVENTORY, r.reason);
        return HIGGS_HAND_OK;
    }
    if (r.kind == HIGGS_TX_EXECUTION_FAILED)
    {
        if(t->state.stage==HIGGS_TX_WAIT_COMPENSATION&&t->compensationPermit)t->compensationReceipt=r;
        else if(t->state.stage==HIGGS_TX_EXECUTING&&!t->compensationPermit)t->lastReceipt=r;
        else return HIGGS_HAND_WRONG_PHASE;
        Finish(*t, HIGGS_TX_RECOVERY_REQUIRED, HIGGS_DISPOSITION_UNKNOWN, r.reason);
        t->state.receiptFlags |= HIGGS_TX_MUTATION_UNCERTAIN;
        return HIGGS_HAND_OK;
    }
    if (t->state.stage != HIGGS_TX_EXECUTING || t->compensationPermit)
        return HIGGS_HAND_WRONG_PHASE;
    if (!r.instancePreserved)
        return HIGGS_HAND_INVALID_ARGUMENT;
    const bool equipment =
        t->request.kind == HIGGS_TRANSFER_EQUIP_GAME || t->request.kind == HIGGS_TRANSFER_UNEQUIP_GAME;
    if (equipment)
    {
        const uint32_t expected = t->request.kind == HIGGS_TRANSFER_EQUIP_GAME ? t->request.expectedEquippedFormId : 0;
        if (r.kind != HIGGS_TX_EQUIPMENT_COMMITTED || r.actualEquippedFormId != expected)
            return HIGGS_HAND_CONFLICT;
    }
    if (t->request.kind == HIGGS_TRANSFER_CONSUME_HELD && r.kind != HIGGS_TX_CONSUMPTION_COMMITTED)
        return HIGGS_HAND_CONFLICT;
    if (!equipment && t->request.kind != HIGGS_TRANSFER_CONSUME_HELD && r.kind != HIGGS_TX_INVENTORY_COMMITTED)
        return HIGGS_HAND_CONFLICT;
    if (t->request.kind == HIGGS_TRANSFER_INVENTORY_TO_HAND && !r.actualRefId)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (r.executionProxy.instance && (!IdentityLocked(r.executionProxy) || r.executionProxy.provider != broker_.ClientLabel(owner)))
        return HIGGS_HAND_STALE;
    if (r.display.instance && (!IdentityLocked(r.display) || r.display.provider != broker_.ClientLabel(owner)))
        return HIGGS_HAND_STALE;
    t->lastReceipt = r;
    t->state.executionProxy = r.executionProxy;
    t->state.display = r.display;
    t->state.actualRefId = r.actualRefId;
    t->state.receiptFlags |= HIGGS_TX_DOMAIN_ATTESTED | HIGGS_TX_INSTANCE_PRESERVED;
    if (t->request.kind == HIGGS_TRANSFER_INVENTORY_TO_HAND)
        if (auto item = FindItem(t->state.original.instance))
            item->available = false;
    broker_.DomainResult(t->state.requestId, HIGGS_ACTION_COMMITTED, equipment ? 0u : HIGGS_RECEIPT_INVENTORY_COMMITTED,
                         r.actualRefId, r.quantity);
    if (equipment)
    {
        t->state.actualEquippedFormId = r.actualEquippedFormId;
        t->equipmentObservation = broker_.CopyPublished().observationId;
        t->state.stage = HIGGS_TX_WAIT_EQUIPMENT;
        return HIGGS_HAND_OK;
    }
    if (t->request.kind == HIGGS_TRANSFER_INVENTORY_TO_HAND)
    {
        t->state.stage = HIGGS_TX_WAIT_GRIP;
        t->state.disposition = HIGGS_DISPOSITION_WORLD;
        t->state.physicalRequestId = 0;
    }
    else
        Finish(*t, HIGGS_TX_COMPLETED,
               t->request.kind == HIGGS_TRANSFER_CONSUME_HELD ? HIGGS_DISPOSITION_CONSUMED
                                                              : HIGGS_DISPOSITION_INVENTORY,
               0);
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus TransactionService::Copy(uint64_t owner, uint64_t id, HIGGS_TransferState &out)
{
    auto phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    SyncLocked();
    if (!broker_.ClientValid(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    auto t = Find(id);
    if (!t)
        return HIGGS_HAND_NOT_FOUND;
    if (t->state.owner != owner)
        return HIGGS_HAND_WRONG_OWNER;
    out = t->state;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus TransactionService::Cancel(uint64_t owner, uint64_t id, uint64_t epoch)
{
    auto phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    SyncLocked();
    if (!broker_.ClientValid(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    auto t = Find(id);
    if (!t)
        return HIGGS_HAND_NOT_FOUND;
    if (t->state.owner != owner)
        return HIGGS_HAND_WRONG_OWNER;
    if (epoch != epoch_ || epoch != t->state.epoch)
        return HIGGS_HAND_STALE;
    if (TerminalTx(t->state.stage))
        return HIGGS_HAND_OK;
    if (t->state.stage == HIGGS_TX_WAIT_COMPENSATION)
        return HIGGS_HAND_OK;
    if (t->state.receiptFlags & HIGGS_TX_DOMAIN_ATTESTED)
        NeedCompensation(*t, HIGGS_HAND_CONFLICT, Now());
    else
    {
        broker_.CancelNativeRequest(owner, t->state.physicalRequestId);
        if (t->permitIssued)
            t->state.receiptFlags |= HIGGS_TX_MUTATION_UNCERTAIN;
        Finish(*t, t->permitIssued ? HIGGS_TX_RECOVERY_REQUIRED : HIGGS_TX_CANCELLED,
               t->permitIssued
                   ? HIGGS_DISPOSITION_UNKNOWN
                   : ((t->state.receiptFlags & HIGGS_TX_NATIVE_RELEASE_CONFIRMED) ? HIGGS_DISPOSITION_WORLD
                                                                                  : HIGGS_DISPOSITION_UNCHANGED),
               0);
    }
    return HIGGS_HAND_OK;
}
void TransactionService::Drain(uint64_t now)
{
    auto phase = broker_.NativePhase();
    for (size_t pass = 0; pass < 128; ++pass)
    {
        HIGGS_TransferExecution execution{};
        bool callback = false;
        {
            std::lock_guard lock(mutex_);
            SyncLocked();
            for (auto &t : transactions_)
            {
                if (!t.state.requestId || TerminalTx(t.state.stage))
                    continue;
                const bool comp = t.state.stage == HIGGS_TX_WAIT_COMPENSATION;
                if (t.state.epoch != epoch_ || !broker_.ClientValid(t.state.owner))
                {
                    Finish(t, HIGGS_TX_RECOVERY_REQUIRED, HIGGS_DISPOSITION_UNKNOWN, HIGGS_HAND_STALE);
                    continue;
                }
                if (now >= t.deadline || (!comp && !Alive(t, now)))
                {
                    if ((t.state.receiptFlags & HIGGS_TX_DOMAIN_ATTESTED) && !comp)
                        NeedCompensation(t, HIGGS_HAND_STALE, now);
                    else
                    {
                        if (t.permitIssued)
                            t.state.receiptFlags |= HIGGS_TX_MUTATION_UNCERTAIN;
                        Finish(t, t.permitIssued || comp ? HIGGS_TX_RECOVERY_REQUIRED : HIGGS_TX_CANCELLED,
                               t.permitIssued || comp ? HIGGS_DISPOSITION_UNKNOWN : t.state.disposition,
                               HIGGS_HAND_STALE);
                    }
                    continue;
                }
                if (t.state.stage == HIGGS_TX_WAIT_EQUIPMENT)
                {
                    HIGGS_HandSnapshot snapshot{};
                    if (!broker_.Copy(t.state.owner, snapshot, now))
                    {
                        const auto &h = snapshot.hands[t.request.destinationHand == HIGGS_HAND_LEFT ? 0 : 1];
                        if (snapshot.observationId > t.equipmentObservation &&
                            (h.observationMask & HIGGS_OBS_EQUIPMENT_KNOWN) &&
                            h.equippedFormId == t.state.actualEquippedFormId && !h.heldRefId)
                        {
                            t.state.receiptFlags |= HIGGS_TX_NATIVE_EQUIPMENT_CONFIRMED;
                            Finish(t, HIGGS_TX_COMPLETED,
                                   t.state.actualEquippedFormId ? HIGGS_DISPOSITION_GAME_EQUIPPED
                                                                : HIGGS_DISPOSITION_INVENTORY,
                                   0);
                        }
                    }
                    continue;
                }
                if (t.state.stage == HIGGS_TX_WAIT_RELEASE || t.state.stage == HIGGS_TX_WAIT_GRIP)
                {
                    const bool release = t.state.stage == HIGGS_TX_WAIT_RELEASE;
                    if (!t.state.physicalRequestId)
                    {
                        HIGGS_HandSnapshot s{};
                        auto status = broker_.Copy(t.state.owner, s, now);
                        if (status)
                            continue;
                        const uint32_t hand = release ? t.request.sourceHand : t.request.destinationHand;
                        const auto &h = s.hands[hand == HIGGS_HAND_LEFT ? 0 : 1];
                        HIGGS_PhysicalRequest p{};
                        p.structSize = sizeof(p);
                        p.kind = release ? HIGGS_PHYSICAL_RELEASE : HIGGS_PHYSICAL_GRAB;
                        p.physicalHand = hand;
                        p.itemRefId = release
                                          ? t.state.original.worldRefId
                                          : (t.request.kind == HIGGS_TRANSFER_HAND_TO_HAND ? t.state.original.worldRefId
                                                                                           : t.state.actualRefId);
                        p.epoch = t.state.epoch;
                        p.handGeneration = h.generation;
                        p.grant = t.state.grant;
                        p.timeoutMs = t.request.timeoutMs;
                        uint64_t id = 0;
                        status = broker_.Physical(t.state.owner, p, id, now);
                        if (status)
                        {
                            if (t.state.receiptFlags & HIGGS_TX_DOMAIN_ATTESTED)
                                NeedCompensation(t, status, now);
                            else
                                Finish(t, HIGGS_TX_REJECTED,
                                       release ? HIGGS_DISPOSITION_UNCHANGED : HIGGS_DISPOSITION_WORLD, status);
                        }
                        else
                        {
                            t.state.physicalRequestId = id;
                            broker_.RetainRequest(id, true);
                        }
                        continue;
                    }
                    HIGGS_ActionResult result{};
                    auto status = broker_.Result(t.state.owner, t.state.physicalRequestId, result);
                    if (status || result.stage == HIGGS_ACTION_REJECTED || result.stage == HIGGS_ACTION_CANCELLED)
                    {
                        if (t.state.receiptFlags & HIGGS_TX_DOMAIN_ATTESTED)
                            NeedCompensation(t, status ? status : result.reason, now);
                        else
                            Finish(t, HIGGS_TX_REJECTED, HIGGS_DISPOSITION_WORLD, status ? status : result.reason);
                        continue;
                    }
                    if (result.stage != HIGGS_ACTION_COMPLETED ||
                        !(result.receiptFlags & HIGGS_RECEIPT_PHYSICAL_CONFIRMED))
                        continue;
                    broker_.RetainRequest(t.state.physicalRequestId, false);
                    t.state.physicalRequestId = 0;
                    if (release)
                    {
                        t.state.receiptFlags |= HIGGS_TX_NATIVE_RELEASE_CONFIRMED;
                        t.state.disposition = HIGGS_DISPOSITION_WORLD;
                        if (t.request.kind == HIGGS_TRANSFER_HAND_TO_HAND)
                        {
                            t.state.stage = HIGGS_TX_WAIT_GRIP;
                            t.state.actualRefId = t.state.original.worldRefId;
                        }
                        else
                            t.state.stage = HIGGS_TX_WAIT_EXECUTOR;
                    }
                    else
                    {
                        t.state.receiptFlags |= HIGGS_TX_NATIVE_GRIP_CONFIRMED;
                        Finish(t, HIGGS_TX_COMPLETED, HIGGS_DISPOSITION_HAND, 0);
                    }
                    continue;
                }
                if ((t.state.stage == HIGGS_TX_WAIT_EXECUTOR || t.state.stage == HIGGS_TX_WAIT_COMPENSATION) &&
                    !t.notified)
                {
                    if (!broker_.ExecutorValid(t.state.owner, t.request.executor, 4))
                    {
                        Finish(t,
                               t.state.receiptFlags & HIGGS_TX_DOMAIN_ATTESTED ? HIGGS_TX_RECOVERY_REQUIRED
                                                                               : HIGGS_TX_REJECTED,
                               HIGGS_DISPOSITION_UNKNOWN, HIGGS_HAND_NOT_FOUND);
                        continue;
                    }
                    t.notified = true;
                    if (t.state.stage == HIGGS_TX_WAIT_EXECUTOR)
                        t.state.stage = HIGGS_TX_EXECUTING;
                    execution.structSize = sizeof(execution);
                    execution.phase =
                        t.state.stage == HIGGS_TX_WAIT_COMPENSATION ? HIGGS_TX_COMPENSATE : HIGGS_TX_EXECUTE;
                    execution.requestId = t.state.requestId;
                    execution.executor = t.request.executor;
                    execution.epoch = t.state.epoch;
                    execution.request = t.request;
                    execution.current = t.state;
                    callback = true;
                    break;
                }
            }
        }
        if (!callback)
            break;
        if (!broker_.InvokeTransfer(execution.current.owner, execution.executor, execution))
        {
            std::lock_guard lock(mutex_);
            if (auto t = Find(execution.requestId))
            {
                if (t->permitIssued)
                    t->state.receiptFlags |= HIGGS_TX_MUTATION_UNCERTAIN;
                Finish(*t, t->permitIssued ? HIGGS_TX_RECOVERY_REQUIRED : HIGGS_TX_REJECTED, HIGGS_DISPOSITION_UNKNOWN,
                       HIGGS_HAND_INTERNAL_ERROR);
            }
        }
    }
}
} // namespace HiggsHands
