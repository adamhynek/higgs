#include "hand_parts_native.h"
#include "hand_parts.h"
#include "hand_broker.h"
#include "hand.h"
#include "main.h"
#include "physics.h"
#include "utils.h"
#include "RE/offsets.h"
#include "controller_tracking.h"
#include <atomic>
#include <cmath>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>
namespace HiggsHands {
namespace {
constexpr unsigned pairLimit=128;
struct Body {uint32_t id=0;uint64_t generation=0;float half[3]{};NiPointer<bhkBoxShape> shape;NiPointer<bhkRigidBody> wrapper;bool addPending=false,removeIssued=false,invalidated=false,addSubmitted=false;unsigned pairSlot=pairLimit;};
struct Rig {uint64_t set=0,epoch=0,revision=0,owner=0;uint32_t hand=0,count=0;NiPointer<bhkWorld> world;std::array<Body,8> bodies{};};
struct Identity {const hkpRigidBody* body=nullptr;NiPointer<bhkBoxShape> retainedShape;NiPointer<bhkRigidBody> retained;uint32_t hand=0,part=0,bodyKey=0;uint64_t set=0,revision=0,epoch=0,owner=0,generation=0;};
struct Model {NiPointer<NiAVObject> hand;NiPointer<bhkRigidBody> palm;std::array<NiPointer<NiAVObject>,15> bones;uint64_t generation=0,epoch=0;};
std::array<Rig,2> rigs;
std::array<Rig,4> retiringRigs;
std::array<bool,4> retirementReserved{};
struct DetachedPair {NiPointer<bhkBoxShape> shape;NiPointer<bhkRigidBody> body;bool reserved=false;};
std::array<DetachedPair,pairLimit> detachedPairs;
struct PairReservation {
    std::array<unsigned,8> slots{};unsigned count=0,used=0;bool ready=false;
    explicit PairReservation(unsigned needed){for(unsigned i=0;i<pairLimit&&count<needed;++i)if(!detachedPairs[i].reserved)slots[count++]=i;ready=count==needed;if(ready)for(unsigned i=0;i<count;++i)detachedPairs[slots[i]].reserved=true;}
    ~PairReservation(){if(ready)for(unsigned i=used;i<count;++i)detachedPairs[slots[i]].reserved=false;}
    unsigned Take(){return slots[used++];}
};
void DrainDetachedPairs(){for(auto& p:detachedPairs)if(p.reserved&&(p.body||p.shape)){if(p.body&&p.body->m_uiRefCount!=1)continue;p.body=nullptr;p.shape=nullptr;p.reserved=false;}}
std::array<Model,2> models;
std::array<HIGGS_HandPartSources,2> stepSources;
const auto emptyIdentities=std::make_shared<const std::vector<Identity>>();
std::atomic<std::shared_ptr<const std::vector<Identity>>> identities{emptyIdentities};
NiPointer<bhkWorld> stepWorld;
uint64_t nextSample=1,nextBody=1;
float stepDt=0;
bool applying=false;
std::thread::id mainThread;
std::atomic<bool> wrongThread=false;
void PublishIdentities()noexcept{try{auto result=std::make_shared<std::vector<Identity>>();const auto append=[&](const auto& collection){for(const auto& rig:collection)for(unsigned i=0;i<rig.count;++i){const auto& b=rig.bodies[i];if(b.wrapper&&b.wrapper->hkBody&&!b.addPending&&!b.invalidated)result->push_back({b.wrapper->hkBody.val(),b.shape,b.wrapper,rig.hand,b.id,b.wrapper->hkBody->m_uid,rig.set,rig.revision,rig.epoch,rig.owner,b.generation});}};append(rigs);append(retiringRigs);identities.store(std::shared_ptr<const std::vector<Identity>>(result));}catch(...){identities.store(emptyIdentities);}}
// Called only after actual detach (or before any add was submitted). External
// native references can outlive both wrappers, so retire only our back-pointers.
// The paired shape wrapper remains retained through body-wrapper destruction.
void ReleaseDetachedBody(Body& b){if(b.wrapper&&b.wrapper->hkBody&&b.wrapper->hkBody->m_userData==reinterpret_cast<uintptr_t>(static_cast<bhkRigidBody*>(b.wrapper)))b.wrapper->hkBody->m_userData=0;if(b.shape&&b.shape->shape&&b.shape->shape->getUserData()==reinterpret_cast<uintptr_t>(static_cast<bhkBoxShape*>(b.shape)))b.shape->shape->setUserData(0);if(b.pairSlot<pairLimit){auto& pair=detachedPairs[b.pairSlot];pair.shape=std::move(b.shape);pair.body=std::move(b.wrapper);if(!pair.body&&!pair.shape)pair.reserved=false;}b={};}
// A submitted add is never resubmitted or assumed cancelled because getWorld is
// still null. A submitted remove retains its wrapper until actual detach, and is
// never reissued. Both operations may be deferred by Havok's critical scope.
bool RemoveLocked(Rig& rig){bool done=true;for(unsigned i=0;i<rig.count;++i){auto& b=rig.bodies[i];if(!b.wrapper||!b.wrapper->hkBody){if(b.addSubmitted){done=false;continue;}ReleaseDetachedBody(b);continue;}auto* native=b.wrapper->hkBody.val();auto* actual=native->getWorld();if(b.addPending){if(rig.world&&actual==rig.world->world.val())b.addPending=false;else{done=false;continue;}}if(!actual){ReleaseDetachedBody(b);continue;}if(!rig.world||!rig.world->world||actual!=rig.world->world.val()){done=false;continue;}if(!b.removeIssued&&!actual->areCriticalOperationsLocked()){b.removeIssued=true;hkBool removed;hkpWorld_RemoveEntity(actual,&removed,native);}if(!native->getWorld())ReleaseDetachedBody(b);else done=false;}if(done)rig={};return done;}
struct RetirementReservation {
    unsigned slots[2]{4,4};
    RetirementReservation(){for(unsigned i=0,n=0;i<4&&n<2;++i)if(!retiringRigs[i].set&&!retirementReserved[i])slots[n++]=i;if(slots[1]<4)for(auto i:slots)retirementReserved[i]=true;}
    ~RetirementReservation(){if(Ready())for(auto i:slots)retirementReserved[i]=false;}
    bool Ready()const{return slots[1]<4;}
    void Park(Rig& rig,unsigned which){retiringRigs[slots[which]]=std::move(rig);rig={};}
};
struct RigCleanup {Rig* previous;Rig* next;RetirementReservation& reservation;~RigCleanup(){if(previous&&!RemoveLocked(*previous))reservation.Park(*previous,0);if(next&&!RemoveLocked(*next))reservation.Park(*next,1);PublishIdentities();}void Release(){previous=next=nullptr;}};
bool RemoveRig(Rig& rig){if(!rig.count){rig={};return true;}if(!rig.world)return false;NiPointer<bhkWorld> retainedWorld=rig.world;BSWriteLocker lock(&retainedWorld->worldLock);const bool done=RemoveLocked(rig);PublishIdentities();return done;}
bool Retiring(uint64_t set){for(const auto& rig:retiringRigs)if(rig.set==set&&set)return true;return false;}
void DrainRetirements(){for(unsigned i=0;i<4;++i)if(!retirementReserved[i]&&retiringRigs[i].set)RemoveRig(retiringRigs[i]);}
HIGGS_HandStatus ResolveBorrowedBody(uint64_t key,HIGGS_HandPartAttribution& out){const auto copy=identities.load();for(const auto& item:*copy)if(reinterpret_cast<uintptr_t>(item.body)==key){out={sizeof(out),item.hand,item.part,item.bodyKey,item.epoch,item.generation,item.set,item.revision,item.owner};return HIGGS_HAND_OK;}return HIGGS_HAND_NOT_FOUND;}
bool SameModel(unsigned index){auto* hand=index==0?g_leftHand:g_rightHand;if(!hand||!(*g_thePlayer))return false;const auto& model=models[index];if(hand->GetFirstPersonHandNode()!=model.hand||hand->handBody!=model.palm)return false;for(unsigned finger=0;finger<5;++finger)for(unsigned joint=0;joint<3;++joint){auto* node=model.hand?model.hand->GetObjectByName(&hand->fingerNodeNames[finger][joint].data):nullptr;if(node!=model.bones[finger*3+joint])return false;}return true;}
void CopyPose(float* p,float* r,const hkTransform& t){for(unsigned i=0;i<3;++i){p[i]=t.getTranslation()(i)* *g_inverseHavokWorldScale;for(unsigned j=0;j<3;++j)r[i*3+j]=t.getRotation().getColumn(j)(i);}}
bool Pose(HIGGS_HandPartSource& out,const NiTransform& t){out.flags=HIGGS_PART_SOURCE_VALID;out.modelScale=t.scale;out.position[0]=t.pos.x;out.position[1]=t.pos.y;out.position[2]=t.pos.z;for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)out.rotation[i*3+j]=t.rot.data[i][j];return true;}
HIGGS_HandStatus CreateBody(bhkWorld* world,const HIGGS_HandPartDefinition& d,const float* p,const float* r,uint32_t filter,Body& output){
    if(world->world->areCriticalOperationsLocked())return HIGGS_HAND_PENDING;auto* rawShape=static_cast<bhkBoxShape*>(Heap_Allocate(sizeof(bhkBoxShape)));if(!rawShape)return HIGGS_HAND_INTERNAL_ERROR;hkVector4 half;half.set(d.halfExtentsMetres[0],d.halfExtentsMetres[1],d.halfExtentsMetres[2],0);bhkBoxShape_ctor(rawShape,&half);output.shape=rawShape;auto& shape=output.shape;if(!shape->shape)return HIGGS_HAND_INTERNAL_ERROR;shape->materialId=0x233db702;static_cast<hkpBoxShape*>(shape->shape.val())->m_radius=0;
    bhkRigidBodyCinfo info;bhkRigidBodyCinfo_ctor(&info);info.collisionFilterInfo=filter;info.hkCinfo.m_collisionFilterInfo=filter;info.shape=shape->shape;info.hkCinfo.m_shape=shape->shape;info.hkCinfo.m_motionType=hkpMotion::MOTION_KEYFRAMED;info.hkCinfo.m_enableDeactivation=false;info.hkCinfo.m_solverDeactivation=hkpRigidBodyCinfo::SOLVER_DEACTIVATION_OFF;info.hkCinfo.m_qualityType=HK_COLLIDABLE_QUALITY_KEYFRAMED;info.hkCinfo.m_maxAngularVelocity=500.f;info.hkCinfo.m_position.set(p[0]* *g_havokWorldScale,p[1]* *g_havokWorldScale,p[2]* *g_havokWorldScale,0);
    NiMatrix33 rotation;for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)rotation.data[i][j]=r[i*3+j];hkRotation hkRotation;NiMatrixToHkMatrix(rotation,hkRotation);info.hkCinfo.m_rotation.setFromRotationSimd(hkRotation);
    auto* rawBody=static_cast<bhkRigidBody*>(Heap_Allocate(sizeof(bhkRigidBody)));if(!rawBody)return HIGGS_HAND_INTERNAL_ERROR;bhkRigidBody_ctor(rawBody,&info);output.wrapper=rawBody;if(!output.wrapper->hkBody)return HIGGS_HAND_INTERNAL_ERROR;bhkRigidBody_setActivated(output.wrapper,true);if(world->world->areCriticalOperationsLocked())return HIGGS_HAND_PENDING;output.addSubmitted=output.addPending=true;hkpWorld_AddEntity(world->world,output.wrapper->hkBody,HK_ENTITY_ACTIVATION_DO_ACTIVATE);if(output.wrapper->hkBody->getWorld()==world->world.val())output.addPending=false;return output.addPending?HIGGS_HAND_PENDING:HIGGS_HAND_OK;
}
HIGGS_HandStatus Drive(const HIGGS_HandPartSetRequest& request,const HIGGS_HandPartSources& source,HIGGS_HandPartSetState& state,bool cleanup,void*){
    auto& rig=rigs[state.physicalHand-1];if(cleanup){const bool removed=rig.set!=state.setId||RemoveRig(rig);return removed&&!Retiring(state.setId)?HIGGS_HAND_OK:HIGGS_HAND_PENDING;}
    if(!applying||!stepWorld||!stepWorld->world||stepDt<=0)return HIGGS_HAND_PENDING;const auto index=state.physicalHand-1;const auto tracking=HiggsMotion::Tracking().CopyObservation();if(source.sampleId!=stepSources[index].sampleId||source.epoch!=stepSources[index].epoch||!SameModel(index)||tracking.generation!=source.trackingGeneration||!tracking.hands[index].available||tracking.hands[index].lastAcceptedObservation!=source.trackingObservationId)return HIGGS_HAND_STALE;auto* hand=index==0?g_leftHand:g_rightHand;auto palm=hand?hand->handBody:nullptr;if(!palm||!palm->hkBody)return HIGGS_HAND_NOT_READY;
    if(Retiring(state.setId))return HIGGS_HAND_PENDING;if(rig.set&&(rig.set!=state.setId||rig.epoch!=state.epoch||rig.world!=stepWorld)&&!RemoveRig(rig))return HIGGS_HAND_PENDING;
    float positions[8][3]{},rotations[8][9]{};for(unsigned i=0;i<request.partCount;++i)if(!HandPartTarget(source,request.parts[i],positions[i],rotations[i]))return HIGGS_HAND_NOT_READY;
    // Remove attribution before topology changes. Retained snapshots keep old body
    // memory alive for readers; no registry mutex is taken by contact callbacks.
    RetirementReservation reservation;if(!reservation.Ready())return HIGGS_HAND_PENDING;BSWriteLocker lock(&stepWorld->worldLock);if(stepWorld->world->areCriticalOperationsLocked())return HIGGS_HAND_PENDING;if(palm->hkBody->getWorld()!=stepWorld->world.val())return HIGGS_HAND_STALE;
    // Validate the complete old topology before moving a single body or issuing
    // any new keyframe/filter/create operation. A matching ID/extents cannot make
    // an externally detached/replaced body a valid native member again.
    bool valid=true;for(unsigned i=0;i<rig.count;++i){auto& b=rig.bodies[i];if(!b.wrapper||!b.wrapper->hkBody||!b.shape||!b.shape->shape||b.addPending||b.removeIssued||b.invalidated||b.wrapper->hkBody->getWorld()!=stepWorld->world.val()||b.wrapper->hkBody->getCollidable()->getShape()!=b.shape->shape.val()){b.invalidated=true;valid=false;}}if(!valid){PublishIdentities();return HIGGS_HAND_STALE;}
    unsigned needed=0;for(unsigned i=0;i<request.partCount;++i){bool reusable=false;for(unsigned j=0;j<rig.count;++j)if(rig.bodies[j].id==request.parts[i].partId&&!std::memcmp(rig.bodies[j].half,request.parts[i].halfExtentsMetres,sizeof(rig.bodies[j].half)))reusable=true;if(!reusable)++needed;}PairReservation pairs(needed);if(!pairs.ready)return HIGGS_HAND_PENDING;
    auto previous=std::move(rig);rig={};PublishIdentities();
    const uint32_t filter=palm->hkBody->getCollisionFilterInfo();Rig next;next.set=state.setId;next.epoch=state.epoch;next.hand=state.physicalHand;next.revision=previous.revision;next.owner=state.owner;next.world=stepWorld;next.count=request.partCount;
    RigCleanup rollback{&previous,&next,reservation};
    for(unsigned i=0;i<request.partCount;++i){auto& b=next.bodies[i];const auto& d=request.parts[i];for(unsigned j=0;j<previous.count;++j){auto& old=previous.bodies[j];if(old.id==d.partId&&!std::memcmp(old.half,d.halfExtentsMetres,sizeof(old.half))&&!old.addPending&&!old.removeIssued){b=std::move(old);old={};break;}}if(!b.wrapper){if(!nextBody||nextBody==UINT64_MAX)return HIGGS_HAND_CAPACITY;b.pairSlot=pairs.Take();b.id=d.partId;b.generation=nextBody++;std::memcpy(b.half,d.halfExtentsMetres,sizeof(b.half));const auto created=CreateBody(stepWorld,d,positions[i],rotations[i],filter|(1u<<14),b);if(created)return created;}}
    const bool removed=RemoveLocked(previous);if(!removed)reservation.Park(previous,0);if(!removed){rig=std::move(next);rollback.Release();PublishIdentities();return HIGGS_HAND_PENDING;}
    for(unsigned i=0;i<request.partCount;++i){auto& b=next.bodies[i];
        auto* body=b.wrapper->hkBody.val();if(body->getCollisionFilterInfo()!=filter){body->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo=filter;hkpWorld_UpdateCollisionFilterOnEntity(stepWorld->world,body,HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK,HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);}NiMatrix33 nr;for(unsigned row=0;row<3;++row)for(unsigned col=0;col<3;++col)nr.data[row][col]=rotations[i][row*3+col];hkRotation hr;NiMatrixToHkMatrix(nr,hr);hkQuaternion q;q.setFromRotationSimd(hr);hkVector4 p;p.set(positions[i][0]* *g_havokWorldScale,positions[i][1]* *g_havokWorldScale,positions[i][2]* *g_havokWorldScale,0);ApplyHardKeyframeVelocityClamped(p,q,1.f/stepDt,b.wrapper);
        auto& out=state.bodies[i];out={};out.partId=b.id;out.flags=HIGGS_PART_BODY_PRESENT|HIGGS_PART_KEYFRAME_ISSUED|HIGGS_PART_ACTUAL_POSE;if(!(filter&(1u<<14)))out.flags|=HIGGS_PART_COLLISION_ENABLED;out.bodyKey=body->m_uid;out.bodyGeneration=b.generation;CopyPose(out.position,out.rotation,body->getTransform());std::memcpy(out.targetPosition,positions[i],sizeof(out.targetPosition));std::memcpy(out.targetRotation,rotations[i],sizeof(out.targetRotation));RegisterPlayerSpaceBody(b.wrapper);
    }
    next.revision=state.requestRevision;rig=std::move(next);rollback.Release();state.bodyCount=request.partCount;PublishIdentities();return HIGGS_HAND_OK;
}
void Capture(bhkWorld* world,unsigned index,const HIGGS_HandSnapshot& snapshot,const HiggsMotion::CopiedTracking& tracking,uint64_t sample,uint64_t at){
    auto* hand=index==0?g_leftHand:g_rightHand;auto& model=models[index];Model current;current.epoch=snapshot.epoch;if(hand&&*g_thePlayer){current.hand=hand->GetFirstPersonHandNode();current.palm=hand->handBody;if(current.hand)for(unsigned finger=0;finger<5;++finger)for(unsigned joint=0;joint<3;++joint)current.bones[finger*3+joint]=current.hand->GetObjectByName(&hand->fingerNodeNames[finger][joint].data);}
    bool changed=current.epoch!=model.epoch||current.hand!=model.hand||current.palm!=model.palm;for(unsigned i=0;i<15;++i)changed=changed||current.bones[i]!=model.bones[i];if(changed||!model.generation){if(model.generation==UINT64_MAX)return;current.generation=model.generation+1;model=std::move(current);}
    HIGGS_HandPartSources out{};out.structSize=sizeof(out);out.physicalHand=index+1;out.sourceCount=17;out.epoch=snapshot.epoch;out.sampleId=sample;out.observedAtNs=at;out.modelGeneration=model.generation;out.handGeneration=snapshot.hands[index].generation;out.trackingGeneration=tracking.generation;out.trackingObservationId=tracking.hands[index].lastAcceptedObservation;out.trackedAtNs=tracking.hands[index].observedAtNs;for(unsigned i=0;i<17;++i)out.sources[i].sourceId=i;
    if(snapshot.worldReady&&(snapshot.hands[index].observationMask&HIGGS_OBS_TRACKING_VALID)&&tracking.worldObserved&&!tracking.loading&&!tracking.exhausted&&tracking.hands[index].available&&at>=out.trackedAtNs&&at-out.trackedAtNs<=100000000&&world&&world->world){BSReadLocker worldLock(&world->worldLock);if(model.palm&&model.palm->hkBody&&model.palm->hkBody->getWorld()==world->world.val()){auto& s=out.sources[0];s.flags=1;s.modelScale=1;CopyPose(s.position,s.rotation,model.palm->hkBody->getTransform());}if(model.hand)Pose(out.sources[1],model.hand->m_worldTransform);for(unsigned i=0;i<15;++i)if(model.bones[i])Pose(out.sources[i+2],model.bones[i]->m_worldTransform);}
    if(HandParts().Publish(out))stepSources[index]=out;
}
}
void MaintainNativeHandParts(){auto phase=Service().NativePhase();if(mainThread==std::thread::id{})mainThread=std::this_thread::get_id();if(mainThread!=std::this_thread::get_id()){wrongThread=true;return;}HandParts().Install(Drive,nullptr,31);InstallHandPartNativeResolver(ResolveBorrowedBody);if(wrongThread.exchange(false)){stepWorld=nullptr;HandParts().Invalidate(HIGGS_PART_REASON_TRACKING);}DrainRetirements();HandParts().Drain(Now());DrainDetachedPairs();const auto registerBodies=[](const auto& collection){for(const auto& rig:collection)for(unsigned i=0;i<rig.count;++i)if(rig.bodies[i].wrapper&&!rig.bodies[i].addPending)RegisterPlayerSpaceBody(rig.bodies[i].wrapper);};registerBodies(rigs);registerBodies(retiringRigs);}
void BeginNativeHandPartsStep(bhkWorld* world,float dt){auto phase=Service().NativePhase();if(mainThread!=std::this_thread::get_id()){wrongThread=true;return;}stepWorld=nullptr;stepDt=0;applying=false;if(!world||!world->world||!std::isfinite(dt)||dt<=0||dt>.25f||!nextSample||nextSample==UINT64_MAX)return;auto* player=*g_thePlayer;auto current=player&&player->parentCell?GetHavokWorldFromCell(player->parentCell):nullptr;if(current!=world)return;const auto snapshot=Service().CopyPublished();if(!snapshot.epoch)return;stepWorld=world;stepDt=dt;const auto sample=nextSample++;const auto tracking=HiggsMotion::Tracking().CopyObservation();const auto at=Now();Capture(world,0,snapshot,tracking,sample,at);Capture(world,1,snapshot,tracking,sample,at);}
void ApplyNativeHandPartsStep(bhkWorld* world,float dt){auto phase=Service().NativePhase();if(mainThread!=std::this_thread::get_id()){wrongThread=true;return;}if(world!=stepWorld||dt!=stepDt||!stepWorld)return;applying=true;try{DrainRetirements();HandParts().Drain(Now());DrainDetachedPairs();}catch(...){applying=false;stepWorld=nullptr;HandParts().Invalidate(HIGGS_PART_REASON_NATIVE);return;}applying=false;stepWorld=nullptr;}
bool ResolveNativeHandPart(const hkpRigidBody* body,uint32_t& hand,uint32_t& part,uint64_t& set,uint64_t& revision){if(!body)return false;const auto copy=identities.load();const auto epoch=Service().Epoch();for(const auto& item:*copy)if(item.body==body&&item.epoch==epoch){hand=item.hand;part=item.part;set=item.set;revision=item.revision;return true;}return false;}
void RecordNativeHandPartContact(const hkpRigidBody* body,uint64_t epoch,uint64_t sequence){if(!body||!epoch||!sequence)return;const auto copy=identities.load();for(const auto& item:*copy)if(item.body==body&&item.epoch==epoch){HIGGS_HandPartContact contact{};contact.structSize=sizeof(contact);contact.contactSequence=sequence;contact.part={sizeof(contact.part),item.hand,item.part,item.bodyKey,item.epoch,item.generation,item.set,item.revision,item.owner};HandParts().Contact(contact);return;}}
}
