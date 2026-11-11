#include "synergy_inventory_native.h"
#include "hand_broker.h"
#include "RE/offsets.h"
#include "utils.h"
#include "skse64/GameEvents.h"
#include "skse64/GameData.h"
#include "skse64/GameRTTI.h"
#include <algorithm>
#include <atomic>
#include <limits>
#include <vector>

namespace HiggsHands {
bool CopyNativeItemReference(const HIGGS_ItemIdentity&,NiPointer<TESObjectREFR>&);
namespace {
struct ContainerFact {uint64_t serial=0;TESContainerChangedEvent value{};};
struct UniqueFact {uint64_t serial=0;TESUniqueIDChangeEvent value{};};
class InventoryEvents final:public BSTEventSink<TESContainerChangedEvent>,public BSTEventSink<TESUniqueIDChangeEvent> {
  public:
    EventResult ReceiveEvent(TESContainerChangedEvent* event,EventDispatcher<TESContainerChangedEvent>*) override {
        if(event){std::lock_guard lock(mutex);const auto serial=++version;facts[serial%facts.size()]={serial,*event};}
        return kEvent_Continue;
    }
    EventResult ReceiveEvent(TESUniqueIDChangeEvent* event,EventDispatcher<TESUniqueIDChangeEvent>*) override {
        if(event){std::lock_guard lock(mutex);const auto serial=++version;uniqueFacts[serial%uniqueFacts.size()]={serial,*event};}return kEvent_Continue;
    }
    uint64_t Version(){std::lock_guard lock(mutex);return version;}
    bool Remap(uint64_t after,uint32_t base,uint32_t& owner,uint16_t& id){
        std::lock_guard lock(mutex);if(version<after||version-after>=uniqueFacts.size())return false;
        for(uint64_t serial=after+1;serial<=version;++serial){const auto& fact=uniqueFacts[serial%uniqueFacts.size()];if(fact.serial!=serial)continue;const auto& e=fact.value;if(e.formId==base&&e.oldOwnerFormId==owner&&e.oldUniqueId==id){owner=e.newOwnerFormId;id=e.newUniqueId;if(!id)return false;}}
        return true;
    }
    bool Changes(uint64_t after,uint32_t player,uint32_t base,uint32_t ref,uint32_t& in,uint32_t& out){
        std::lock_guard lock(mutex);in=out=0;
        if(version<after||version-after>=facts.size())return false;
        for(const auto& fact:facts)if(fact.serial>after&&fact.value.itemFormId==base){
            const auto& e=fact.value;
            if((e.fromFormId==player||e.toFormId==player)&&e.toReference&&ref&&e.toReference!=ref)return false;
            if(e.toFormId==player){if(UINT32_MAX-in<e.count)return false;in+=e.count;}
            if(e.fromFormId==player){if(UINT32_MAX-out<e.count)return false;out+=e.count;}
        }
        return true;
    }
  private:
    std::mutex mutex;uint64_t version=1;std::array<ContainerFact,128> facts{};std::array<UniqueFact,128> uniqueFacts{};
};
InventoryEvents events;
struct LiveInventory {
    ExtraContainerChanges::Data* changes=nullptr;
    int64_t total=0,plain=0;
    std::vector<BaseExtraList*> lists;
};
bool Ordinary(TESForm* form){
    if(!form||!form->IsPlayable())return false;
    switch(form->formType){
        case kFormType_Weapon:case kFormType_Armor:case kFormType_Ammo:case kFormType_Book:
        case kFormType_Potion:case kFormType_Ingredient:case kFormType_Misc:case kFormType_Key:
        case kFormType_SoulGem:case kFormType_ScrollItem:return true;
        default:return false;
    }
}
bool Alive(TESObjectREFR* reference){return reference&&!(reference->flags&TESForm::kFlagIsDeleted);}
bool ReadInventory(TESObjectREFR* player,TESForm* base,LiveInventory& out){
    if(!Alive(player)||!base)return false;
    if(auto container=DYNAMIC_CAST(player->baseForm,TESForm,TESContainer))out.total=container->CountItem(base);
    auto extra=static_cast<ExtraContainerChanges*>(player->extraData.GetByType(kExtraData_ContainerChanges));
    out.changes=extra?extra->data:nullptr;
    auto entry=out.changes?out.changes->FindItemEntry(base):nullptr;
    if(entry){
        out.total+=entry->countDelta;
        if(entry->extendDataList){
            size_t visited=0;
            for(auto it=entry->extendDataList->Begin();!it.End();++it){
                if(++visited>4096)return false;
                if(auto list=it.Get())out.lists.push_back(list);
            }
        }
    }
    if(out.total<0||out.total>INT32_MAX)return false;
    out.plain=out.total;
    for(auto list:out.lists){auto count=BSExtraList_GetCount(list);if(!count||count>INT32_MAX)return false;out.plain-=count;}
    return out.plain>=0;
}
struct Unique {uint32_t owner=0;uint16_t id=0;};
Unique UID(BaseExtraList* list){
    if(list)if(auto id=static_cast<ExtraUniqueID*>(list->GetByType(kExtraData_UniqueID)))return {id->ownerFormId,id->uniqueId};
    return {};
}
bool Same(Unique a,Unique b){return a.owner==b.owner&&a.id==b.id;}
bool Worn(BaseExtraList* list,bool left){return list&&list->HasType(left?kExtraData_WornLeft:kExtraData_Worn);}
struct NativeSelection final:InventorySelectionHandle {
    NiPointer<TESObjectREFR> player;
    TESForm* base=nullptr;
    uintptr_t locator=0; // An identity number only. Never dereference this value.
    Unique unique{};
    uint64_t version=0;
    int64_t total=0;
    uint32_t quantity=0,stackCount=0;
    bool worn=false,wornLeft=false;
};
bool Resolve(const NativeSelection& s,LiveInventory& live,BaseExtraList*& selected){
    selected=nullptr;
    if(!s.player||s.player!=*g_thePlayer||events.Version()!=s.version||!ReadInventory(s.player,s.base,live)||live.total!=s.total)return false;
    if(!s.locator)return live.plain==s.stackCount&&live.plain>=s.quantity;
    size_t matches=0;
    for(auto list:live.lists){
        if(s.unique.id?Same(UID(list),s.unique):reinterpret_cast<uintptr_t>(list)==s.locator){selected=list;++matches;}
    }
    return matches==1&&BSExtraList_GetCount(selected)==s.stackCount&&s.stackCount>=s.quantity&&Worn(selected,false)==s.worn&&Worn(selected,true)==s.wornLeft;
}
bool HasCustomConsumptionData(BaseExtraList& list){
    // A plain potion/ingredient can merge into a homogeneous stack. An instance with
    // meaningful data needs a persistent UID to identify the new inventory list.
    for(auto type:{kExtraData_Enchantment,kExtraData_Health,kExtraData_Charge,kExtraData_Poison,
                   kExtraData_Ownership,kExtraData_TextDisplayData,kExtraData_Soul})if(list.HasType(type))return true;
    return false;
}
BaseExtraList* FindUID(const LiveInventory& live,Unique unique){
    BaseExtraList* found=nullptr;
    if(!unique.id)return nullptr;
    for(auto list:live.lists)if(Same(UID(list),unique)){if(found)return nullptr;found=list;}
    return found;
}
bool HandEquipment(TESForm* base){
    if(!base)return false;
    if(base->formType==kFormType_Weapon||base->formType==kFormType_ScrollItem)return true;
    if(auto armor=DYNAMIC_CAST(base,TESForm,TESObjectARMO))return (armor->bipedObject.GetSlotMask()&BGSBipedObjectForm::kPart_Shield)!=0;
    return false;
}
struct Operation {
    uint64_t request=0,version=0;
    uint32_t kind=0,quantity=0,hand=0;
    UInt32 droppedHandle=0;
    NiPointer<TESObjectREFR> player,reference;
    TESForm* base=nullptr;
    std::shared_ptr<NativeSelection> selection;
    int64_t beforeTotal=0;
    Unique unique{};
    uintptr_t locator=0;
    bool compensation=false,consumed=false,left=false;
};
bool CrossHandEquipment(const Operation& op,const LiveInventory& live,BaseExtraList* selected){
    if(selected)return Worn(selected,!op.left);
    // A plain stack has no exact equipped-list identity. Do not let the engine
    // choose an already worn same-base instance from the opposite hand.
    auto actor=static_cast<Actor*>(static_cast<TESObjectREFR*>(op.player));
    if(actor->GetEquippedObject(!op.left)==op.base)return true;
    for(auto list:live.lists)if(Worn(list,!op.left))return true;
    return false;
}
class NativeInventory final:public InventoryEngine {
  public:
    uint64_t Capabilities() const override{return HIGGS_INVENTORY_CAP_PLAYER_SELECTION|HIGGS_INVENTORY_CAP_DROP_TO_HAND|HIGGS_INVENTORY_CAP_STORE_HELD|HIGGS_INVENTORY_CAP_CONSUME_HELD|HIGGS_INVENTORY_CAP_GAME_EQUIPMENT;}
    HIGGS_HandStatus Select(const HIGGS_InventorySelectionRequest& r,std::shared_ptr<InventorySelectionHandle>& out,uint32_t& container) override {
        auto player=*g_thePlayer;auto base=LookupFormByID(r.baseFormId);
        if(!player||!Service().MutationAdmissionReady())return HIGGS_HAND_NOT_READY;
        if(!Ordinary(base))return HIGGS_HAND_UNSUPPORTED;
        LiveInventory live{};if(!ReadInventory(player,base,live))return HIGGS_HAND_NOT_READY;
        auto selection=std::make_shared<NativeSelection>();selection->player=player;selection->base=base;selection->quantity=r.quantity;selection->total=live.total;selection->version=events.Version();
        if(r.stackOrdinal){
            if(r.stackOrdinal>live.lists.size())return HIGGS_HAND_NOT_FOUND;
            auto list=live.lists[r.stackOrdinal-1];selection->locator=reinterpret_cast<uintptr_t>(list);selection->unique=UID(list);selection->stackCount=BSExtraList_GetCount(list);selection->worn=Worn(list,false);selection->wornLeft=Worn(list,true);
            if(selection->unique.id&&FindUID(live,selection->unique)!=list)return HIGGS_HAND_CONFLICT;
        }else selection->stackCount=static_cast<uint32_t>(live.plain);
        if(selection->stackCount<r.quantity)return HIGGS_HAND_NOT_FOUND;
        container=player->formID;out=selection;return HIGGS_HAND_OK;
    }
    bool Current(const std::shared_ptr<InventorySelectionHandle>& handle) override {
        auto s=std::dynamic_pointer_cast<NativeSelection>(handle);LiveInventory live{};BaseExtraList* list=nullptr;return s&&Resolve(*s,live,list);
    }
    InventoryOutcome Execute(const HIGGS_TransferExecution& x,const std::shared_ptr<InventorySelectionHandle>& handle,InventoryPermit& permit) override {
        if(x.phase==HIGGS_TX_COMPENSATE)return Compensate(x,permit);
        if(Find(x.requestId))return {HIGGS_HAND_CONFLICT};
        Operation op{};op.request=x.requestId;op.kind=x.request.kind;op.quantity=x.request.quantity;op.hand=x.request.destinationHand;op.player=*g_thePlayer;op.base=LookupFormByID(x.request.original.baseFormId);op.version=events.Version();
        if(!op.player||!Ordinary(op.base)||!Service().MutationAdmissionReady())return {HIGGS_HAND_UNSUPPORTED};
        auto bound=DYNAMIC_CAST(op.base,TESForm,TESBoundObject);if(!bound)return {HIGGS_HAND_UNSUPPORTED};
        BaseExtraList* list=nullptr;LiveInventory before{};
        const bool inventory=op.kind==HIGGS_TRANSFER_INVENTORY_TO_HAND||op.kind==HIGGS_TRANSFER_EQUIP_GAME||op.kind==HIGGS_TRANSFER_UNEQUIP_GAME;
        if(inventory){
            op.selection=std::dynamic_pointer_cast<NativeSelection>(handle);
            if(!op.selection||op.selection->quantity!=op.quantity||op.selection->base!=op.base||!Resolve(*op.selection,before,list))return {HIGGS_HAND_STALE};
            op.unique=op.selection->unique;op.locator=op.selection->locator;
            if(op.kind==HIGGS_TRANSFER_INVENTORY_TO_HAND&&(op.selection->worn||op.selection->wornLeft))return {HIGGS_HAND_CONFLICT};
        }else{
            if(op.kind!=HIGGS_TRANSFER_HAND_TO_INVENTORY&&op.kind!=HIGGS_TRANSFER_CONSUME_HELD)return {HIGGS_HAND_UNSUPPORTED};
            if(!CopyNativeItemReference(x.request.original,op.reference)||!Alive(op.reference)||op.reference->baseForm!=op.base||BSExtraList_GetCount(&op.reference->extraData)!=op.quantity||!ReadInventory(op.player,op.base,before))return {HIGGS_HAND_STALE};
            op.unique=UID(&op.reference->extraData);
            if(op.kind==HIGGS_TRANSFER_CONSUME_HELD){
                if(op.quantity!=1||(op.base->formType!=kFormType_Potion&&op.base->formType!=kFormType_Ingredient))return {HIGGS_HAND_UNSUPPORTED};
                if(auto potion=DYNAMIC_CAST(op.base,TESForm,AlchemyItem))if(potion->IsPoison())return {HIGGS_HAND_UNSUPPORTED};
                if(!op.unique.id&&HasCustomConsumptionData(op.reference->extraData))return {HIGGS_HAND_UNSUPPORTED};
            }
        }
        if(op.kind==HIGGS_TRANSFER_EQUIP_GAME||op.kind==HIGGS_TRANSFER_UNEQUIP_GAME){
            if(!HandEquipment(op.base)||op.quantity!=1||x.request.expectedEquippedFormId!=op.base->formID)return {HIGGS_HAND_UNSUPPORTED};
            auto weapon=DYNAMIC_CAST(op.base,TESForm,TESObjectWEAP);
            if(weapon&&IsTwoHanded(weapon)&&x.current.physicalHands!=HIGGS_HAND_BOTH)return {HIGGS_HAND_CONFLICT};
            if(weapon&&IsTwoHanded(weapon)){
                const auto physical=IsBow(weapon)?(*g_leftHandedMode?HIGGS_HAND_RIGHT:HIGGS_HAND_LEFT):(*g_leftHandedMode?HIGGS_HAND_LEFT:HIGGS_HAND_RIGHT);
                if(op.hand!=static_cast<uint32_t>(physical))return {HIGGS_HAND_CONFLICT};
            }
            op.left=*g_leftHandedMode!=(op.hand==HIGGS_HAND_LEFT);
            if(weapon&&IsTwoHanded(weapon))op.left=false;
            if(!EquipManager::GetSingleton())return {HIGGS_HAND_NOT_READY};
            if(op.kind==HIGGS_TRANSFER_UNEQUIP_GAME&&(!list||!Worn(list,op.left)))return {HIGGS_HAND_STALE};
            if(op.kind==HIGGS_TRANSFER_EQUIP_GAME&&CrossHandEquipment(op,before,list))return {HIGGS_HAND_UNSUPPORTED};
        }
        if(Conflicts(op))return {HIGGS_HAND_CONFLICT};
        op.beforeTotal=before.total;
        if(!permit.Begin())return {HIGGS_HAND_STALE};
        // No engine pointer from a previous frame is used. Resolve once more after
        // the one-shot permit, immediately before the only mutation call.
        if(inventory){
            LiveInventory fresh{};if(!Resolve(*op.selection,fresh,list))return {HIGGS_HAND_STALE};
            if(op.kind==HIGGS_TRANSFER_EQUIP_GAME&&CrossHandEquipment(op,fresh,list))return {HIGGS_HAND_UNSUPPORTED};
        }
        else if(!Alive(op.reference)||op.reference->baseForm!=op.base||BSExtraList_GetCount(&op.reference->extraData)!=op.quantity)return {HIGGS_HAND_STALE};
        operations.push_back(op);auto& saved=operations.back();
        if(op.kind==HIGGS_TRANSFER_INVENTORY_TO_HAND){
            UInt32 dropped=*g_invalidRefHandle;
            NiPoint3 position=op.player->pos;
            const auto snapshot=Service().CopyPublished();const auto& hand=snapshot.hands[op.hand==HIGGS_HAND_LEFT?0:1];
            if(hand.observationMask&HIGGS_OBS_PALM_VALID)position=NiPoint3(hand.palmPosition[0],hand.palmPosition[1],hand.palmPosition[2]);
            get_vfunc<Actor_RemoveItem>(op.player,0x56)(op.player,&dropped,bound,static_cast<SInt32>(op.quantity),3,list,nullptr,&position,nullptr);
            saved.droppedHandle=dropped;
            if(!LookupREFRByHandle(dropped,saved.reference)||!saved.reference)return {HIGGS_HAND_PENDING};
        }else if(op.kind==HIGGS_TRANSFER_HAND_TO_INVENTORY||op.kind==HIGGS_TRANSFER_CONSUME_HELD){
            auto actor=static_cast<Actor*>(static_cast<TESObjectREFR*>(op.player));
            get_vfunc<Actor_PickUpObject>(actor,0xCE)(actor,op.reference,static_cast<SInt32>(op.quantity),false,true);
            if(op.kind==HIGGS_TRANSFER_CONSUME_HELD){
                LiveInventory after{};if(!ReadInventory(op.player,op.base,after)||after.total!=op.beforeTotal+op.quantity)return {HIGGS_HAND_INTERNAL_ERROR};
                auto pickedUnique=op.unique;if(pickedUnique.id&&!events.Remap(op.version,op.base->formID,pickedUnique.owner,pickedUnique.id))return {HIGGS_HAND_INTERNAL_ERROR};
                auto exact=pickedUnique.id?FindUID(after,pickedUnique):nullptr;
                if(op.unique.id&&!exact)return {HIGGS_HAND_INTERNAL_ERROR};
                if(!exact&&after.plain<op.quantity)return {HIGGS_HAND_INTERNAL_ERROR};
                auto manager=EquipManager::GetSingleton();if(!manager)return {HIGGS_HAND_INTERNAL_ERROR};
                CALL_MEMBER_FN(manager,EquipItem)(actor,op.base,exact,static_cast<SInt32>(op.quantity),nullptr,true,false,false,nullptr);
                saved.consumed=true;
            }
        }else ApplyEquipment(saved,list,op.kind==HIGGS_TRANSFER_EQUIP_GAME);
        return Observe(saved);
    }
    InventoryOutcome Poll(const HIGGS_TransferExecution& x) override {auto op=Find(x.requestId);return op?Observe(*op):InventoryOutcome{HIGGS_HAND_STALE};}
    void Forget(uint64_t request) override {operations.erase(std::remove_if(operations.begin(),operations.end(),[&](const Operation& o){return o.request==request;}),operations.end());}
    void Reset() override{operations.clear();}
  private:
    Operation* Find(uint64_t request){for(auto& o:operations)if(o.request==request)return &o;return nullptr;}
    bool Conflicts(const Operation& op){
        for(const auto& existing:operations){
            if(existing.reference&&op.reference&&existing.reference==op.reference)return true;
            if(existing.player==op.player&&existing.base==op.base&&(!existing.reference||!op.reference)){
                // The inventory owner can relocate an extra list without changing
                // the actual UID instance. Address identity alone is not its lock.
                if(existing.locator==op.locator)return true;
                auto before=existing.unique,after=op.unique;
                // Resolve an existing persistent identity even if the incoming
                // ticket no longer has one. UID loss is uncertainty, not proof
                // that a relocated same-base source is independent of this lock.
                if(before.id&&!events.Remap(existing.version,existing.base->formID,before.owner,before.id))return true;
                if(after.id&&!events.Remap(op.version,op.base->formID,after.owner,after.id))return true;
                if(before.id&&after.id){
                    if(Same(before,after))return true;
                }else if(!existing.locator||!op.locator)return true;
            }
        }
        return operations.size()>=128;
    }
    void ApplyEquipment(const Operation& op,BaseExtraList* list,bool equip){
        auto manager=EquipManager::GetSingleton();auto actor=static_cast<Actor*>(static_cast<TESObjectREFR*>(op.player));auto slot=op.left?GetLeftHandSlot():GetRightHandSlot();
        if(equip)CALL_MEMBER_FN(manager,EquipItem)(actor,op.base,list,1,slot,true,false,false,nullptr);
        else CALL_MEMBER_FN(manager,UnequipItem)(actor,op.base,list,1,slot,true,false,true,false,nullptr);
    }
    InventoryOutcome Compensate(const HIGGS_TransferExecution& x,InventoryPermit& permit){
        auto op=Find(x.requestId);if(!op||op->compensation)return {HIGGS_HAND_STALE};
        if(op->kind!=HIGGS_TRANSFER_INVENTORY_TO_HAND&&op->kind!=HIGGS_TRANSFER_EQUIP_GAME&&op->kind!=HIGGS_TRANSFER_UNEQUIP_GAME)return {HIGGS_HAND_UNSUPPORTED};
        LiveInventory live{};if(!op->player||op->player!=*g_thePlayer||!ReadInventory(op->player,op->base,live))return {HIGGS_HAND_STALE};
        BaseExtraList* list=nullptr;
        if(op->kind==HIGGS_TRANSFER_INVENTORY_TO_HAND){if(!Alive(op->reference)||op->reference->baseForm!=op->base||BSExtraList_GetCount(&op->reference->extraData)!=op->quantity)return {HIGGS_HAND_STALE};}
        else {list=op->unique.id?FindUID(live,op->unique):nullptr;if(!op->unique.id)for(auto fresh:live.lists)if(reinterpret_cast<uintptr_t>(fresh)==op->locator)list=fresh;if(!list)return {HIGGS_HAND_STALE};}
        if(!permit.Begin())return {HIGGS_HAND_STALE};
        op->compensation=true;op->version=events.Version();op->beforeTotal=live.total;
        if(op->kind==HIGGS_TRANSFER_INVENTORY_TO_HAND){auto actor=static_cast<Actor*>(static_cast<TESObjectREFR*>(op->player));get_vfunc<Actor_PickUpObject>(actor,0xCE)(actor,op->reference,static_cast<SInt32>(op->quantity),false,true);}
        else ApplyEquipment(*op,list,op->kind==HIGGS_TRANSFER_UNEQUIP_GAME);
        return Observe(*op);
    }
    InventoryOutcome Observe(Operation& op){
        if(!op.player||op.player!=*g_thePlayer)return {HIGGS_HAND_STALE};
        if(!op.reference&&op.kind==HIGGS_TRANSFER_INVENTORY_TO_HAND&&op.droppedHandle&&op.droppedHandle!=*g_invalidRefHandle)LookupREFRByHandle(op.droppedHandle,op.reference);
        LiveInventory live{};if(!ReadInventory(op.player,op.base,live))return {HIGGS_HAND_PENDING};
        uint32_t incoming=0,outgoing=0;
        if(!events.Changes(op.version,op.player->formID,op.base->formID,op.reference?op.reference->formID:0,incoming,outgoing))return {HIGGS_HAND_INTERNAL_ERROR};
        if(op.kind==HIGGS_TRANSFER_EQUIP_GAME||op.kind==HIGGS_TRANSFER_UNEQUIP_GAME){
            BaseExtraList* list=op.unique.id?FindUID(live,op.unique):nullptr;
            if(!op.unique.id&&op.locator)for(auto fresh:live.lists)if(reinterpret_cast<uintptr_t>(fresh)==op.locator)list=fresh;
            const bool equip=(op.kind==HIGGS_TRANSFER_EQUIP_GAME)!=op.compensation;
            auto actor=static_cast<Actor*>(static_cast<TESObjectREFR*>(op.player));
            if(equip&&!list&&!op.locator&&!op.unique.id){
                for(auto fresh:live.lists)if(Worn(fresh,op.left)){if(list)return {HIGGS_HAND_INTERNAL_ERROR};list=fresh;}
                if(list){op.locator=reinterpret_cast<uintptr_t>(list);op.unique=UID(list);}
            }
            if(equip){if(!list||!Worn(list,op.left)||actor->GetEquippedObject(op.left)!=op.base)return {HIGGS_HAND_PENDING};}
            else if((list&&Worn(list,op.left))||actor->GetEquippedObject(op.left)==op.base)return {HIGGS_HAND_PENDING};
            return {HIGGS_HAND_OK,static_cast<uint32_t>(op.compensation?HIGGS_TX_COMPENSATED:HIGGS_TX_EQUIPMENT_COMMITTED),0,equip?static_cast<uint32_t>(op.base->formID):0,true};
        }
        const bool toInventory=op.compensation||op.kind==HIGGS_TRANSFER_HAND_TO_INVENTORY;
        auto expected=op.unique;if(expected.id&&!events.Remap(op.version,op.base->formID,expected.owner,expected.id))return {HIGGS_HAND_INTERNAL_ERROR};
        if(toInventory){
            if(live.total!=op.beforeTotal+op.quantity||incoming!=op.quantity||outgoing||Alive(op.reference))return {HIGGS_HAND_PENDING};
            if(expected.id&&!FindUID(live,expected))return {HIGGS_HAND_PENDING};
            return {HIGGS_HAND_OK,static_cast<uint32_t>(op.compensation?HIGGS_TX_COMPENSATED:HIGGS_TX_INVENTORY_COMMITTED),0,0,true};
        }
        if(op.kind==HIGGS_TRANSFER_CONSUME_HELD){
            if(!op.consumed||live.total!=op.beforeTotal||incoming!=op.quantity||outgoing!=op.quantity||Alive(op.reference))return {HIGGS_HAND_PENDING};
            if(expected.id&&FindUID(live,expected))return {HIGGS_HAND_PENDING};
            return {HIGGS_HAND_OK,HIGGS_TX_CONSUMPTION_COMMITTED,0,0,true};
        }
        if(!Alive(op.reference)||op.reference->baseForm!=op.base||BSExtraList_GetCount(&op.reference->extraData)!=op.quantity||live.total!=op.beforeTotal-op.quantity||outgoing!=op.quantity||incoming)return {HIGGS_HAND_PENDING};
        if(expected.id&&!Same(UID(&op.reference->extraData),expected))return {HIGGS_HAND_INTERNAL_ERROR};
        return {HIGGS_HAND_OK,HIGGS_TX_INVENTORY_COMMITTED,op.reference->formID,0,true};
    }
    std::vector<Operation> operations;
};
NativeInventory nativeInventory;
EventDispatcherList* installed=nullptr;
}
void DrainNativeInventory(){
    auto dispatchers=GetEventDispatcherList();
    if(!dispatchers)return;
    if(dispatchers!=installed){
        // Engine dispatchers live for the process. Registration is main-phase only.
        dispatchers->unk370.AddEventSink(static_cast<BSTEventSink<TESContainerChangedEvent>*>(&events));
        dispatchers->uniqueIdChangeDispatcher.AddEventSink(static_cast<BSTEventSink<TESUniqueIDChangeEvent>*>(&events));
        installed=dispatchers;
    }
    Inventory().Install(&nativeInventory);Inventory().Drain();
}
}
