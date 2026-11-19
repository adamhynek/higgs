#include "own_contact.h"
#include "hand_broker.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
namespace HiggsHands {
namespace {
bool Item(const HIGGS_ContactItem& v){return v.structSize==sizeof(v)&&(v.physicalHand==1||v.physicalHand==2)&&(v.source==1||v.source==2)&&(v.itemClass==1||v.itemClass==2||v.itemClass==4)&&v.baseFormId&&v.itemInstance&&v.itemGeneration&&v.bodyGeneration&&v.handGeneration&&!v.reserved&&((v.source==1&&v.worldRefId&&v.provenance==HIGGS_CONTACT_PROVENANCE_HELD_REFERENCE)||(v.source==2&&!v.worldRefId&&(v.provenance==HIGGS_CONTACT_PROVENANCE_EQUIPPED_UID||v.provenance==HIGGS_CONTACT_PROVENANCE_EQUIPPED_COLLIDER)));}
bool Same(const HIGGS_ContactItem& a,const HIGGS_ContactItem& b){return !std::memcmp(&a,&b,sizeof(a));}
bool Vector(const float* v,float bound){for(unsigned i=0;i<3;++i)if(!std::isfinite(v[i])||std::abs(v[i])>bound)return false;return true;}
}
OwnContactService& OwnContacts(){static OwnContactService service;return service;}
OwnContactService::Lease* OwnContactService::Find(uint64_t id){for(auto& l:leases_)if(id&&l.state.leaseId==id)return &l;return nullptr;}
void OwnContactService::Expire(uint64_t at){for(auto& l:leases_)if(l.state.leaseId&&l.state.stage==HIGGS_COSMETIC_ACTIVE&&(l.state.epoch!=frame_.epoch||at>=l.state.expiresAtNs)){l.state.stage=HIGGS_COSMETIC_ENDED;l.state.reason=HIGGS_HAND_STALE;}}
bool OwnContactService::Current(const HIGGS_ContactItem& item)const{if(!ready_)return false;for(unsigned i=0;i<frame_.count;++i)if(Same(frame_.items[i],item))return true;return false;}
bool OwnContactService::Publish(const OwnContactFrame& frame,uint64_t caps,uint64_t at){
    std::lock_guard lock(mutex_);
    if(frame.invalidationTicket!=invalidationTicket_.load()||frame.invalidationTicket==UINT64_MAX||
       (frame.epoch==frame_.epoch&&frame.observation<=frame_.observation))return false;
    if(frame_.epoch!=frame.epoch){contacts_={};count_=write_=0;}
    frame_=frame;capabilities_=caps;ready_=frame.count<=frame.items.size()&&frame.epoch&&frame.observation&&frame.observedAt;
    if(ready_)for(unsigned i=0;i<frame.count;++i)if(!Item(frame.items[i]))ready_=false;
    Expire(at);
    return ready_;
}
void OwnContactService::Invalidate(){std::lock_guard lock(mutex_);ready_=false;const auto ticket=invalidationTicket_.load();if(ticket!=UINT64_MAX)invalidationTicket_.store(ticket+1,std::memory_order_release);}
uint64_t OwnContactService::Capabilities(){std::lock_guard lock(mutex_);return capabilities_;}
HIGGS_HandStatus OwnContactService::Items(uint64_t owner,HIGGS_ContactItemFrame& out){auto phase=Service().NativePhase();if(!Service().ClientValid(owner))return HIGGS_HAND_INVALID_CLIENT;const auto epoch=Service().Epoch();std::lock_guard lock(mutex_);if(!ready_)return HIGGS_HAND_NOT_READY;if(epoch!=frame_.epoch)return HIGGS_HAND_STALE;out={};out.structSize=sizeof(out);out.itemCount=frame_.count;out.epoch=frame_.epoch;out.observationId=frame_.observation;out.observedAtNs=frame_.observedAt;for(unsigned i=0;i<4;++i){out.sources[i]=frame_.sources[i];out.items[i]=frame_.items[i];}return HIGGS_HAND_OK;}
HIGGS_HandStatus OwnContactService::Request(uint64_t owner,const HIGGS_CosmeticContactPolicy& p,HIGGS_CosmeticContactState& out,uint64_t at){
    auto phase=Service().NativePhase();if(!Service().ClientValid(owner))return HIGGS_HAND_INVALID_CLIENT;const auto epoch=Service().Epoch();
    if(p.structSize!=sizeof(p)||p.reserved||!p.provenanceMask||(p.provenanceMask&~7u)||!p.feedbackKinds||(p.feedbackKinds&~3u)||!p.leftSources||(p.leftSources&~3u)||!p.rightSources||(p.rightSources&~3u)||!p.leftClasses||(p.leftClasses&~7u)||!p.rightClasses||(p.rightClasses&~7u)||!std::isfinite(p.minRelativeSpeed)||p.minRelativeSpeed<0||p.minRelativeSpeed>1000||!std::isfinite(p.minClosingSpeed)||p.minClosingSpeed<0||p.minClosingSpeed>1000||p.cooldownMs>60000||!p.leaseMs||p.leaseMs>60000||!p.maxObservationAgeMs||p.maxObservationAgeMs>100||at>UINT64_MAX-uint64_t(p.leaseMs)*1000000)return HIGGS_HAND_INVALID_ARGUMENT;
    std::lock_guard lock(mutex_);Expire(at);if(p.epoch!=epoch||p.epoch!=frame_.epoch)return HIGGS_HAND_STALE;if(!capabilities_||!ready_)return HIGGS_HAND_NOT_READY;
    Lease* lease=nullptr;
    if(p.leaseId){lease=Find(p.leaseId);if(!lease)return HIGGS_HAND_NOT_FOUND;if(lease->state.owner!=owner)return HIGGS_HAND_WRONG_OWNER;if(lease->state.stage!=HIGGS_COSMETIC_ACTIVE||lease->state.revision!=p.expectedRevision)return HIGGS_HAND_CONFLICT;if(lease->state.revision==UINT64_MAX)return HIGGS_HAND_CAPACITY;}
    else{if(p.expectedRevision)return HIGGS_HAND_INVALID_ARGUMENT;for(auto& l:leases_)if(!l.state.leaseId||l.state.stage==HIGGS_COSMETIC_ENDED){lease=&l;break;}if(!lease||next_==UINT64_MAX)return HIGGS_HAND_CAPACITY;*lease={};lease->state={sizeof(out),HIGGS_COSMETIC_ACTIVE,next_++,owner,p.epoch,0,0,0,0};}
    lease->policy=p;lease->state.revision++;lease->state.expiresAtNs=at+uint64_t(p.leaseMs)*1000000;out=lease->state;return HIGGS_HAND_OK;
}
HIGGS_HandStatus OwnContactService::End(uint64_t owner,uint64_t id,uint64_t epoch,uint64_t at){
    auto phase=Service().NativePhase();if(!Service().ClientValid(owner))return HIGGS_HAND_INVALID_CLIENT;const auto current=Service().Epoch();std::lock_guard lock(mutex_);Expire(at);auto l=Find(id);if(!l)return HIGGS_HAND_NOT_FOUND;if(l->state.owner!=owner)return HIGGS_HAND_WRONG_OWNER;if(epoch!=current||epoch!=l->state.epoch)return HIGGS_HAND_STALE;l->state.stage=HIGGS_COSMETIC_ENDED;return HIGGS_HAND_OK;
}
HIGGS_HandStatus OwnContactService::Copy(uint64_t owner,uint64_t id,HIGGS_CosmeticContactState& out,uint64_t at){
    auto phase=Service().NativePhase();if(!Service().ClientValid(owner))return HIGGS_HAND_INVALID_CLIENT;const auto epoch=Service().Epoch();std::lock_guard lock(mutex_);Expire(at);auto l=Find(id);if(!l)return HIGGS_HAND_NOT_FOUND;if(l->state.owner!=owner)return HIGGS_HAND_WRONG_OWNER;if(l->state.epoch!=epoch){l->state.stage=HIGGS_COSMETIC_ENDED;l->state.reason=HIGGS_HAND_STALE;}out=l->state;return HIGGS_HAND_OK;
}
HIGGS_HandStatus OwnContactService::Contacts(uint64_t owner,const HIGGS_CosmeticContactQuery& q,HIGGS_CosmeticContact* out,uint32_t capacity,uint32_t& required,uint64_t at){
    auto phase=Service().NativePhase();if(!Service().ClientValid(owner))return HIGGS_HAND_INVALID_CLIENT;const auto epoch=Service().Epoch();if(q.structSize!=sizeof(q)||q.reserved||q.reserved2||(q.feedbackKinds&~3u)||(capacity&&!out))return HIGGS_HAND_INVALID_ARGUMENT;
    std::lock_guard lock(mutex_);Expire(at);if(q.epoch!=epoch||q.epoch!=frame_.epoch)return HIGGS_HAND_STALE;if(!ready_)return HIGGS_HAND_NOT_READY;
    if(q.leaseId){auto l=Find(q.leaseId);if(!l)return HIGGS_HAND_NOT_FOUND;if(l->state.owner!=owner)return HIGGS_HAND_WRONG_OWNER;}
    const auto eligible=[&](const HIGGS_CosmeticContact& c){auto l=Find(c.leaseId);return c.sequence>q.afterSequence&&c.epoch==epoch&&(!q.leaseId||q.leaseId==c.leaseId)&&(!q.feedbackKinds||(q.feedbackKinds&c.feedbackKinds))&&l&&l->state.owner==owner&&l->state.stage==HIGGS_COSMETIC_ACTIVE&&l->state.revision==c.policyRevision&&at>=c.producerObservedAtNs&&at-c.producerObservedAtNs<=uint64_t(l->policy.maxObservationAgeMs)*1000000&&Current(c.left)&&Current(c.right);};
    uint32_t count=0;for(size_t i=0;i<count_;++i){const auto& c=contacts_[(write_+contacts_.size()-count_+i)%contacts_.size()];if(eligible(c))++count;}if(capacity<count){required=count;return HIGGS_HAND_BUFFER_TOO_SMALL;}
    for(uint32_t i=0;i<count;++i)if(out[i].structSize<sizeof(*out))return HIGGS_HAND_INVALID_ARGUMENT;
    required=count;count=0;for(size_t i=0;i<count_;++i){const auto& c=contacts_[(write_+contacts_.size()-count_+i)%contacts_.size()];if(eligible(c))out[count++]=c;}return HIGGS_HAND_OK;
}
HIGGS_HandStatus OwnContactService::Observe(const OwnContactFrame& frame,const HIGGS_ContactItem& first,const HIGGS_ContactItem& second,const HIGGS_NativeOwnContact& input,uint64_t at,uint64_t& last){
    if(input.structSize!=sizeof(input)||input.producer!=HIGGS_NATIVE_OWN_CONTACT_PLANCK||input.reserved||(input.contactFlags&~3u)||!input.bodyA||!input.bodyB||input.bodyA==input.bodyB||!input.world||!input.mainFrame||!input.physicsTick||!Item(first)||!Item(second)||first.physicalHand==second.physicalHand||first.itemInstance==second.itemInstance||(first.source==HIGGS_CONTACT_ITEM_HELD&&second.source==HIGGS_CONTACT_ITEM_HELD&&first.worldRefId==second.worldRefId)||!Vector(input.positionMetres,1000000)||!Vector(input.bToANormal,2)||!Vector(input.pointVelocityA,10000)||!Vector(input.pointVelocityB,10000))return HIGGS_HAND_INVALID_ARGUMENT;
    const bool reverse=first.physicalHand!=HIGGS_HAND_LEFT;const auto& left=reverse?second:first;const auto& right=reverse?first:second;
    const auto* lv=reverse?input.pointVelocityB:input.pointVelocityA;const auto* rv=reverse?input.pointVelocityA:input.pointVelocityB;
    double normalLength=0,speedSquared=0,dot=0;for(unsigned i=0;i<3;++i){normalLength+=double(input.bToANormal[i])*input.bToANormal[i];const auto v=double(lv[i])-rv[i];speedSquared+=v*v;dot+=v*input.bToANormal[i]*(reverse?-1:1);}if(normalLength<0.99||normalLength>1.01)return HIGGS_HAND_INVALID_ARGUMENT;
    const auto speed=static_cast<float>(std::sqrt(speedSquared));const auto closing=static_cast<float>((std::max)(0.,-dot/std::sqrt(normalLength)));
    std::lock_guard lock(mutex_);Expire(at);if(!ready_||!capabilities_)return HIGGS_HAND_NOT_READY;if(frame.invalidationTicket!=invalidationTicket_.load()||frame.invalidationTicket!=frame_.invalidationTicket||frame.epoch!=frame_.epoch||frame.observation!=frame_.observation||frame.observedAt!=frame_.observedAt||!Current(left)||!Current(right)||at<frame.observedAt)return HIGGS_HAND_STALE;
    bool recorded=false;
    for(auto& l:leases_){const auto& p=l.policy;if(l.state.stage!=HIGGS_COSMETIC_ACTIVE||!(p.provenanceMask&left.provenance)||!(p.provenanceMask&right.provenance)||!(p.leftSources&left.source)||!(p.rightSources&right.source)||!(p.leftClasses&left.itemClass)||!(p.rightClasses&right.itemClass)||speed<p.minRelativeSpeed||closing<p.minClosingSpeed||at-frame.observedAt>uint64_t(p.maxObservationAgeMs)*1000000)continue;
        bool duplicate=false;for(const auto& s:l.recent)if(Same(s.left,left)&&Same(s.right,right)&&((s.frame==input.mainFrame&&s.tick==input.physicsTick)||(at>=s.at&&at-s.at<uint64_t(p.cooldownMs)*1000000))){duplicate=true;break;}if(duplicate)continue;if(sequence_==UINT64_MAX)return HIGGS_HAND_CAPACITY;
        auto& stamp=l.recent[l.next++%l.recent.size()];stamp={left,right,input.mainFrame,input.physicsTick,at};
        HIGGS_CosmeticContact c{};c.structSize=sizeof(c);c.flags=HIGGS_COSMETIC_VERIFIED_PAIR|HIGGS_COSMETIC_SERVICE_NO_DAMAGE;if(input.contactFlags&HIGGS_NATIVE_OWN_RESPONSE_DISABLED)c.flags|=HIGGS_COSMETIC_NATIVE_RESPONSE_DISABLED;if(input.contactFlags&HIGGS_NATIVE_OWN_HIT_PATH_SKIPPED)c.flags|=HIGGS_COSMETIC_PLANCK_HIT_PATH_SKIPPED;c.sequence=sequence_++;c.epoch=frame.epoch;c.leaseId=l.state.leaseId;c.policyRevision=l.state.revision;c.producerObservation=frame.observation;c.producerObservedAtNs=frame.observedAt;c.observedAtNs=at;c.mainFrame=input.mainFrame;c.physicsTick=input.physicsTick;c.left=left;c.right=right;c.relativeSpeed=speed;c.closingSpeed=closing;c.feedbackKinds=p.feedbackKinds;
        for(unsigned i=0;i<3;++i){c.positionMetres[i]=input.positionMetres[i];c.rightToLeftNormal[i]=input.bToANormal[i]*(reverse?-1.f:1.f);c.leftPointVelocity[i]=lv[i];c.rightPointVelocity[i]=rv[i];}
        contacts_[write_++%contacts_.size()]=c;if(count_<contacts_.size())++count_;last=c.sequence;recorded=true;
    }
    return recorded?HIGGS_HAND_OK:HIGGS_HAND_NOT_FOUND;
}
}
