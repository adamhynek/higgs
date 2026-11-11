#include "own_contact.h"
#include "hand_broker.h"
#include "hand.h"
#include "menu_checker.h"
#include "utils.h"
#include "skse64/GameEvents.h"
#include "skse64/GameRTTI.h"
#include <atomic>
#include <memory>
#include <thread>
#include <vector>

// Public CommonLibSSE-NG a898f469... TESEquipEvent data/layout. The legacy
// SDK uses the same trivial, two-slot sink specialization for its named events;
// the generic template only declares its destructor. This defines our local
// concrete observer interface, not an engine function or SDK replacement.
struct HiggsOwnEquipEvent {
    NiPointer<TESObjectREFR> actor;
    uint32_t baseObject,originalReference;uint16_t uniqueID;bool equipped;
    uint8_t pad13;uint32_t pad14;
};
template<> class BSTEventSink<HiggsOwnEquipEvent> {
public:
    virtual ~BSTEventSink()=default;
    virtual EventResult ReceiveEvent(HiggsOwnEquipEvent*,EventDispatcher<HiggsOwnEquipEvent>*)=0;
};
static_assert(sizeof(HiggsOwnEquipEvent)==0x18);
static_assert(sizeof(BSTEventSink<HiggsOwnEquipEvent>)==sizeof(void*));

namespace HiggsHands {
namespace {
// Shape wrapper precedes body wrapper so the body is destroyed before its
// corresponding shape wrapper, also in retained immutable reader snapshots.
struct Source {
    NiPointer<TESObjectREFR> reference;
    NiPointer<NiAVObject> model;
    NiPointer<bhkWorld> world;
    NiPointer<bhkShape> shapeWrapper;
    NiPointer<bhkRigidBody> wrapper;
    RE::hkRefPtr<hkpShape> shape;
    RE::hkRefPtr<hkpRigidBody> body;
    HIGGS_ContactItem item{};
    uint64_t epoch=0,episode=0,equipmentVersion=0,worldKey=0;
    uint32_t uidOwner=0;uint16_t uid=0;
};
struct Snapshot {OwnContactFrame frame;std::array<Source,4> sources;uint64_t version=0;};
const auto emptySnapshot=std::make_shared<const Snapshot>();
std::atomic<std::shared_ptr<const Snapshot>> snapshot{emptySnapshot};
std::atomic<uint64_t> equipmentVersion{1};
std::atomic<uintptr_t> playerIdentity{0};
std::array<Source,4> previous;
uint64_t nextIdentity=1,nextObservation=1;
std::thread::id publishThread;

using EquipEvent=HiggsOwnEquipEvent;
static_assert(sizeof(EquipEvent)==0x18);
static_assert(offsetof(EventDispatcherList,unk4D0)==0x478);
struct Events final:BSTEventSink<EquipEvent>,BSTEventSink<TESContainerChangedEvent>,BSTEventSink<TESUniqueIDChangeEvent> {
    EventResult ReceiveEvent(EquipEvent* value,EventDispatcher<EquipEvent>*)override{
        if(value&&reinterpret_cast<uintptr_t>(static_cast<TESObjectREFR*>(value->actor))==playerIdentity.load()){
            equipmentVersion.fetch_add(1);InvalidateNativeOwnContacts();
        }return kEvent_Continue;
    }
    EventResult ReceiveEvent(TESContainerChangedEvent*,EventDispatcher<TESContainerChangedEvent>*)override{InvalidateNativeOwnContacts();return kEvent_Continue;}
    EventResult ReceiveEvent(TESUniqueIDChangeEvent*,EventDispatcher<TESUniqueIDChangeEvent>*)override{equipmentVersion.fetch_add(1);InvalidateNativeOwnContacts();return kEvent_Continue;}
} events;
EventDispatcherList* installed=nullptr;

uint32_t Class(TESForm* base){
    if(!base||!base->IsPlayable())return 0;
    if(base->formType==kFormType_Weapon)return HIGGS_CONTACT_CLASS_WEAPON;
    if(auto armor=DYNAMIC_CAST(base,TESForm,TESObjectARMO))return (armor->bipedObject.GetSlotMask()&BGSBipedObjectForm::kPart_Shield)?HIGGS_CONTACT_CLASS_SHIELD:HIGGS_CONTACT_CLASS_OTHER;
    switch(base->formType){case kFormType_Ammo:case kFormType_Book:case kFormType_Potion:case kFormType_Ingredient:case kFormType_Misc:case kFormType_Key:case kFormType_SoulGem:case kFormType_ScrollItem:return HIGGS_CONTACT_CLASS_OTHER;default:return 0;}
}
uint32_t WornUID(PlayerCharacter* player,TESForm* base,bool left,uint32_t& owner,uint16_t& uid){
    auto changes=static_cast<ExtraContainerChanges*>(player->extraData.GetByType(kExtraData_ContainerChanges));
    auto entry=changes&&changes->data?changes->data->FindItemEntry(base):nullptr;
    if(!entry||!entry->extendDataList)return HIGGS_CONTACT_SOURCE_UNSUPPORTED;
    BaseExtraList* worn=nullptr;unsigned visits=0,matches=0;
    for(auto it=entry->extendDataList->Begin();!it.End();++it){if(++visits>4096)return HIGGS_CONTACT_SOURCE_AMBIGUOUS;auto list=it.Get();if(list&&list->HasType(left?kExtraData_WornLeft:kExtraData_Worn)){worn=list;++matches;}}
    if(matches!=1)return matches?HIGGS_CONTACT_SOURCE_AMBIGUOUS:HIGGS_CONTACT_SOURCE_UNSUPPORTED;
    auto value=static_cast<ExtraUniqueID*>(worn->GetByType(kExtraData_UniqueID));if(!value||!value->uniqueId||BSExtraList_GetCount(worn)!=1)return HIGGS_CONTACT_SOURCE_UNSUPPORTED;
    owner=static_cast<uint32_t>(value->ownerFormId);uid=value->uniqueId;matches=0;
    for(auto it=entry->extendDataList->Begin();!it.End();++it)if(auto list=it.Get())if(auto id=static_cast<ExtraUniqueID*>(list->GetByType(kExtraData_UniqueID)))if(id->ownerFormId==owner&&id->uniqueId==uid)++matches;
    return matches==1?HIGGS_CONTACT_SOURCE_AVAILABLE:HIGGS_CONTACT_SOURCE_AMBIGUOUS;
}
bool SameItem(const Source& a,const Source& b){return a.epoch==b.epoch&&a.item.source==b.item.source&&a.item.provenance==b.item.provenance&&a.item.baseFormId==b.item.baseFormId&&a.reference==b.reference&&a.model==b.model&&a.episode==b.episode&&a.uidOwner==b.uidOwner&&a.uid==b.uid&&a.equipmentVersion==b.equipmentVersion;}
bool SameBody(const Source& a,const Source& b){return a.epoch==b.epoch&&a.world==b.world&&a.wrapper==b.wrapper&&a.body==b.body&&a.shapeWrapper==b.shapeWrapper&&a.shape==b.shape;}
bool RetainBody(Source& out,bhkWorld* world,bhkRigidBody* wrapper){
    if(!world||!world->world||!wrapper||!wrapper->hkBody)return false;
    out.world=world;out.wrapper=wrapper;out.body=wrapper->hkBody;
    out.worldKey=reinterpret_cast<uintptr_t>(world->world.val());
    if(out.body->getWorld()!=world->world.val()||(out.body->getCollisionFilterInfo()&(1u<<14)))return false;
    out.shape=const_cast<hkpShape*>(out.body->getCollidable()->getShape());if(!out.shape||!out.shape->getUserData())return false;
    out.shapeWrapper=reinterpret_cast<bhkShape*>(out.shape->getUserData());return out.shapeWrapper&&out.shapeWrapper->shape==out.shape;
}
uint32_t Capture(Source& out,unsigned handIndex,bool equipped,const HIGGS_HandSnapshot& sample,bhkWorld* world){
    auto hand=handIndex==0?g_leftHand:g_rightHand;auto player=*g_thePlayer;const auto& record=sample.hands[handIndex];
    if(!hand||!player||!(record.observationMask&HIGGS_OBS_TRACKING_VALID))return HIGGS_CONTACT_SOURCE_NOT_READY;
    out.epoch=sample.epoch;out.item.structSize=sizeof(out.item);out.item.physicalHand=handIndex+1;out.item.handGeneration=record.generation;out.item.source=equipped?HIGGS_CONTACT_ITEM_EQUIPPED:HIGGS_CONTACT_ITEM_HELD;
    TESForm* base=nullptr;
    if(!equipped){
        if(!record.heldRefId||!hand->HasHeldObject())return HIGGS_CONTACT_SOURCE_ABSENT;
        if(hand->synergyContactHoldEpisode==UINT64_MAX||!hand->synergyContactHoldEpisode||!LookupREFRByHandle(hand->selectedObject.handle,out.reference)||!out.reference||out.reference->formID!=record.heldRefId)return HIGGS_CONTACT_SOURCE_NOT_READY;
        base=out.reference->baseForm;out.model=out.reference->GetNiNode();out.episode=hand->synergyContactHoldEpisode;out.item.worldRefId=record.heldRefId;out.item.provenance=HIGGS_CONTACT_PROVENANCE_HELD_REFERENCE;
        if(!out.model||!RetainBody(out,world,hand->selectedObject.rigidBody))return HIGGS_CONTACT_SOURCE_NOT_READY;
    }else{
        if(!(record.observationMask&HIGGS_OBS_EQUIPMENT_KNOWN))return HIGGS_CONTACT_SOURCE_NOT_READY;
        if(!record.equippedFormId)return HIGGS_CONTACT_SOURCE_ABSENT;
        if(!player->actorState.IsWeaponDrawn()||hand->IsTwoHanding())return HIGGS_CONTACT_SOURCE_NOT_READY;
        const bool left=*g_leftHandedMode!=(handIndex==0);base=player->GetEquippedObject(left);
        if(!base||base->formID!=record.equippedFormId)return HIGGS_CONTACT_SOURCE_NOT_READY;
        if(auto weapon=DYNAMIC_CAST(base,TESForm,TESObjectWEAP))if(IsTwoHanded(weapon)||weapon->gameData.type==TESObjectWEAP::GameData::kType_HandToHandMelee)return HIGGS_CONTACT_SOURCE_UNSUPPORTED;
        const auto kind=Class(base);if(kind!=HIGGS_CONTACT_CLASS_WEAPON&&kind!=HIGGS_CONTACT_CLASS_SHIELD)return HIGGS_CONTACT_SOURCE_UNSUPPORTED;
        const auto uidStatus=WornUID(player,base,left,out.uidOwner,out.uid);
        if(uidStatus==HIGGS_CONTACT_SOURCE_AVAILABLE)out.item.provenance=HIGGS_CONTACT_PROVENANCE_EQUIPPED_UID;
        else{out.uid=0;out.uidOwner=0;out.item.provenance=HIGGS_CONTACT_PROVENANCE_EQUIPPED_COLLIDER;}
        auto data=GetVRMeleeData(handIndex==0);if(!data||!data->collisionNode)return HIGGS_CONTACT_SOURCE_NOT_READY;
        auto original=GetRigidBody(data->collisionNode);if(!original||!original->hkBody||!original->hkBody->getCollidable()->getShape())return HIGGS_CONTACT_SOURCE_NOT_READY;
        const auto shape=original->hkBody->getCollidable()->getShape();const Hand::ShapeIdentifier key{original,original->hkBody.val(),reinterpret_cast<void*>(shape->getUserData()),shape};
        if(!hand->clonedFromWeaponShape||!(*hand->clonedFromWeaponShape==key))return HIGGS_CONTACT_SOURCE_NOT_READY;
        out.reference=player;out.model=data->collisionNode;out.equipmentVersion=equipmentVersion.load();
        if(!RetainBody(out,world,hand->weaponBody))return HIGGS_CONTACT_SOURCE_NOT_READY;
    }
    const auto kind=Class(base);if(!kind)return HIGGS_CONTACT_SOURCE_UNSUPPORTED;out.item.itemClass=kind;out.item.baseFormId=static_cast<uint32_t>(base->formID);return HIGGS_CONTACT_SOURCE_AVAILABLE;
}
HIGGS_HandStatus Observe(const HIGGS_NativeOwnContact& input,uint64_t& last){
    const auto copy=snapshot.load(std::memory_order_acquire);if(copy->version!=OwnContacts().SourceTicket()||!copy->frame.epoch)return HIGGS_HAND_NOT_READY;
    const Source* a=nullptr;const Source* b=nullptr;
    for(const auto& source:copy->sources){
        if(!source.body||source.worldKey!=input.world)continue;
        if(reinterpret_cast<uintptr_t>(source.body.val())==input.bodyA){if(a)return HIGGS_HAND_CONFLICT;a=&source;}
        if(reinterpret_cast<uintptr_t>(source.body.val())==input.bodyB){if(b)return HIGGS_HAND_CONFLICT;b=&source;}
    }
    if(!a||!b)return HIGGS_HAND_NOT_FOUND;
    return OwnContacts().Observe(copy->frame,a->item,b->item,input,Now(),last);
}
}
void InvalidateNativeOwnContacts(){OwnContacts().Invalidate();}
void PublishNativeOwnContacts(){
    if(publishThread==std::thread::id{})publishThread=std::this_thread::get_id();if(publishThread!=std::this_thread::get_id()){InvalidateNativeOwnContacts();return;}
    auto dispatchers=GetEventDispatcherList();if(!dispatchers)return;
    if(installed!=dispatchers){
        reinterpret_cast<EventDispatcher<EquipEvent>*>(&dispatchers->unk4D0)->AddEventSink(static_cast<BSTEventSink<EquipEvent>*>(&events));
        dispatchers->unk370.AddEventSink(static_cast<BSTEventSink<TESContainerChangedEvent>*>(&events));dispatchers->uniqueIdChangeDispatcher.AddEventSink(static_cast<BSTEventSink<TESUniqueIDChangeEvent>*>(&events));installed=dispatchers;
    }
    if(nextObservation==UINT64_MAX||OwnContacts().SourceTicket()==UINT64_MAX||equipmentVersion.load()==UINT64_MAX){InvalidateNativeOwnContacts();return;}
    auto player=*g_thePlayer;playerIdentity.store(reinterpret_cast<uintptr_t>(player));const auto sample=Service().CopyPublished();
    auto next=std::make_shared<Snapshot>();next->version=OwnContacts().SourceTicket();next->frame.invalidationTicket=next->version;next->frame.epoch=sample.epoch;next->frame.observation=nextObservation++;next->frame.observedAt=Now();next->frame.sources.fill(HIGGS_CONTACT_SOURCE_NOT_READY);
    NiPointer<bhkWorld> world=player&&player->parentCell?GetHavokWorldFromCell(player->parentCell):nullptr;
    if(sample.worldReady&&world&&world->world&&!MenuChecker::isGameStopped()){
        BSReadLocker lock(&world->worldLock);
        for(unsigned hand=0;hand<2;++hand)for(unsigned kind=0;kind<2;++kind){const auto index=hand*2+kind;auto& source=next->sources[index];const auto status=Capture(source,hand,kind!=0,sample,world);next->frame.sources[index]=status;if(status!=HIGGS_CONTACT_SOURCE_AVAILABLE){source={};continue;}
            const auto& old=previous[index];if(SameItem(source,old)&&old.item.itemInstance){source.item.itemInstance=old.item.itemInstance;source.item.itemGeneration=old.item.itemGeneration;}else{if(nextIdentity>=UINT64_MAX-2){source={};next->frame.sources[index]=HIGGS_CONTACT_SOURCE_NOT_READY;continue;}source.item.itemInstance=nextIdentity++;source.item.itemGeneration=nextIdentity++;}
            if(SameBody(source,old)&&old.item.bodyGeneration)source.item.bodyGeneration=old.item.bodyGeneration;
            else{if(nextIdentity==UINT64_MAX){source={};next->frame.sources[index]=HIGGS_CONTACT_SOURCE_NOT_READY;continue;}source.item.bodyGeneration=nextIdentity++;}
            next->frame.items[next->frame.count++]=source.item;
        }
    }
    if(!OwnContacts().Publish(next->frame,HIGGS_OWN_CONTACT_CAP_HELD|HIGGS_OWN_CONTACT_CAP_EQUIPPED_UID|HIGGS_OWN_CONTACT_CAP_EQUIPPED_COLLIDER,Now()))return;
    previous=next->sources;snapshot.store(std::shared_ptr<const Snapshot>(next),std::memory_order_release);InstallOwnContactNativeObserver(Observe);
}
}
