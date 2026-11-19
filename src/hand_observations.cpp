#include "hand_observations.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
namespace HiggsHands {
namespace {
bool Finite(const float* p,size_t n){for(size_t i=0;i<n;++i)if(!std::isfinite(p[i]))return false;return true;}
bool Zero(const float* p,size_t n){for(size_t i=0;i<n;++i)if(p[i]!=0)return false;return true;}
bool Rotation(const float* r){
    if(!Finite(r,9))return false;
    for(unsigned i=0;i<3;++i)for(unsigned j=i;j<3;++j){double v=0;for(unsigned k=0;k<3;++k)v+=double(r[i*3+k])*r[j*3+k];if(std::abs(v-(i==j?1.:0.))>.001)return false;}
    const double determinant=double(r[0])*(double(r[4])*r[8]-double(r[5])*r[7])-double(r[1])*(double(r[3])*r[8]-double(r[5])*r[6])+double(r[2])*(double(r[3])*r[7]-double(r[4])*r[6]);return std::abs(determinant-1.)<=.001;
}
bool Query(const HIGGS_ObservationQuery& q){return q.structSize==sizeof(q)&&q.epoch&&q.physicalHands&&!(q.physicalHands&~3u);}
bool MotionValid(const HIGGS_MotionSample& v){
    constexpr uint32_t flags=HIGGS_SAMPLE_POSE|HIGGS_SAMPLE_LINEAR|HIGGS_SAMPLE_ANGULAR|HIGGS_SAMPLE_MAPPING|HIGGS_SAMPLE_NO_HARDWARE_TIME;
    if(v.structSize!=sizeof(v)||!v.epoch||!v.observedAtNs||v.physicalHand<1||v.physicalHand>2||v.source<1||v.source>3||v.space<1||v.space>2||v.reserved||v.reserved2||v.status>HIGGS_SAMPLE_DISCONNECTED||(v.flags&~flags)||!(v.flags&HIGGS_SAMPLE_NO_HARDWARE_TIME))return false;
    if(v.source!=HIGGS_SAMPLE_CONTROLLER&&v.space!=HIGGS_SPACE_WORLD)return false;
    if(v.source==HIGGS_SAMPLE_CONTROLLER&&(!v.trackingGeneration||!v.captureId||v.itemRefId||v.bodyKey))return false;
    if(v.source!=HIGGS_SAMPLE_CONTROLLER&&v.deviceIndex)return false;
    if(v.source==HIGGS_SAMPLE_CONTROLLER&&v.status==HIGGS_SAMPLE_AVAILABLE&&(!v.deviceIndex||v.deviceIndex>=64))return false;
    if(v.source==HIGGS_SAMPLE_CONTROLLER&&v.status==HIGGS_SAMPLE_AVAILABLE&&v.space==HIGGS_SPACE_WORLD&&!(v.flags&HIGGS_SAMPLE_MAPPING))return false;
    if((v.flags&HIGGS_SAMPLE_MAPPING)&&(!v.spaceGeneration||v.source!=HIGGS_SAMPLE_CONTROLLER))return false;
    if(!(v.flags&HIGGS_SAMPLE_MAPPING)&&v.spaceGeneration)return false;
    if(v.status!=HIGGS_SAMPLE_AVAILABLE)return !(v.flags&~HIGGS_SAMPLE_NO_HARDWARE_TIME)&&Zero(v.position,3)&&Zero(v.rotation,9)&&Zero(v.linearVelocity,3)&&Zero(v.angularVelocity,3);
    if(!(v.flags&HIGGS_SAMPLE_POSE)||!v.observationId||!v.handGeneration||!Finite(v.position,3)||!Rotation(v.rotation))return false;
    if((v.flags&HIGGS_SAMPLE_LINEAR)?!Finite(v.linearVelocity,3):!Zero(v.linearVelocity,3))return false;
    if((v.flags&HIGGS_SAMPLE_ANGULAR)?!Finite(v.angularVelocity,3):!Zero(v.angularVelocity,3))return false;
    return v.source!=HIGGS_SAMPLE_HELD_BODY||v.itemRefId;
}
bool ContactValid(const HIGGS_ContactObservation& v){
    constexpr uint32_t flags=HIGGS_CONTACT_ENABLED|HIGGS_CONTACT_USED|HIGGS_CONTACT_POTENTIAL|HIGGS_CONTACT_POINT|HIGGS_CONTACT_NORMAL|HIGGS_CONTACT_LINEAR;
    if(v.structSize!=sizeof(v)||!v.epoch||!v.observedAtNs||!v.observationId||!v.physicalHands||(v.physicalHands&~3u)||v.source<1||v.source>3||!(v.flags&HIGGS_CONTACT_ENABLED)||(v.flags&~flags)||v.reserved)return false;
    if((v.flags&HIGGS_CONTACT_POINT)?!Finite(v.position,3):!Zero(v.position,3))return false;
    if(v.flags&HIGGS_CONTACT_NORMAL){if(!Finite(v.targetToProbeNormal,3))return false;double n=0;for(float x:v.targetToProbeNormal)n+=double(x)*x;if(std::abs(n-1.)>.001)return false;}else if(!Zero(v.targetToProbeNormal,3))return false;
    return (v.flags&HIGGS_CONTACT_LINEAR)?Finite(v.probeLinearVelocity,3)&&Finite(v.targetLinearVelocity,3):Zero(v.probeLinearVelocity,3)&&Zero(v.targetLinearVelocity,3);
}
bool MappingValid(const HIGGS_TrackingMappingSample& v){
    return v.structSize==sizeof(v)&&v.flags==HIGGS_SAMPLE_NO_HARDWARE_TIME&&v.epoch&&v.spaceGeneration&&v.trackingGeneration&&v.captureId&&v.observedAtNs&&!v.reserved&&Rotation(v.rotation)&&Finite(v.translation,3)&&std::isfinite(v.unitsPerMeter)&&v.unitsPerMeter>0;
}
template<class T,size_t N>void Append(std::array<T,N>& rows,size_t& count,size_t& cursor,const T& value){rows[cursor]=value;cursor=(cursor+1)%N;if(count<N)++count;}
template<class T,size_t N>
HIGGS_HandStatus Copy(const std::array<T,N>& ring,size_t count,const HIGGS_ObservationQuery& q,T* out,uint32_t cap,uint32_t& needed){
    auto result=std::make_unique<std::array<T,N>>();size_t found=0;
    for(size_t i=0;i<count;++i){const auto& r=ring[i];
        const auto hands=[&](){if constexpr(requires{r.physicalHands;})return r.physicalHands;else if constexpr(requires{r.physicalHand;})return r.physicalHand;else return uint32_t(HIGGS_HAND_BOTH);}();
        const auto space=[&](){if constexpr(requires{r.space;})return r.space;else return uint32_t(HIGGS_SPACE_WORLD);}();
        const auto source=[&](){if constexpr(requires{r.source;})return r.source;else return uint32_t(HIGGS_SAMPLE_CONTROLLER);}();
        if(r.epoch==q.epoch&&(hands&q.physicalHands)&&(!q.source||source==q.source)&&(!q.space||space==q.space)&&r.sequence>q.afterSequence&&r.observedAtNs>=q.minObservedAtNs)(*result)[found++]=r;
    }
    if(cap<found){needed=static_cast<uint32_t>(found);return HIGGS_HAND_BUFFER_TOO_SMALL;}
    std::sort(result->begin(),result->begin()+found,[](const T& a,const T& b){return a.sequence<b.sequence;});
    for(size_t i=0;i<found;++i)out[i]=(*result)[i];needed=static_cast<uint32_t>(found);return HIGGS_HAND_OK;
}
}
void ObservationStore::EpochLocked(uint64_t epoch){
    if(!epoch||epoch<=epoch_)return;epoch_=epoch;
    motionCount_=contactCount_=releaseCount_=mappingCount_=0;motionWrite_=contactWrite_=releaseWrite_=mappingWrite_=0;
    lastTrackingGeneration_=lastCapture_=0;
    motion_={};contacts_={};releases_={};mappings_={};
}
void ObservationStore::Epoch(uint64_t epoch){std::lock_guard lock(mutex_);EpochLocked(epoch);}
bool ObservationStore::Motion(HIGGS_MotionSample value){
    if(!MotionValid(value)||value.source==HIGGS_SAMPLE_CONTROLLER)return false;std::lock_guard lock(mutex_);EpochLocked(value.epoch);
    if(value.epoch!=epoch_||next_==(std::numeric_limits<uint64_t>::max)())return false;
    value.sequence=next_++;Append(motion_,motionCount_,motionWrite_,value);return true;
}
bool ObservationStore::Contact(HIGGS_ContactObservation value,uint64_t* assignedSequence,HIGGS_ContactDetails details){
    if(assignedSequence)*assignedSequence=0;
    if(!ContactValid(value))return false;std::lock_guard lock(mutex_);EpochLocked(value.epoch);
    if(value.epoch!=epoch_||next_==(std::numeric_limits<uint64_t>::max)())return false;
    value.sequence=next_++;
    details.structSize=sizeof(details); details.epoch=value.epoch; details.contactSequence=value.sequence;
    details.probe.observedBodyKey=static_cast<uint32_t>(value.probeBodyKey);
    details.target.observedBodyKey=static_cast<uint32_t>(value.targetBodyKey);
    details_[contactWrite_]=details;
    Append(contacts_,contactCount_,contactWrite_,value);if(assignedSequence)*assignedSequence=value.sequence;return true;
}
HIGGS_HandStatus ObservationStore::CopyContactDetails(uint64_t epoch,uint64_t seq,HIGGS_ContactDetails &out){
    std::lock_guard lock(mutex_);
    if(!epoch||!seq)return HIGGS_HAND_INVALID_ARGUMENT;
    if(epoch!=epoch_)return HIGGS_HAND_STALE;
    for(size_t i=0;i<contactCount_;++i)
        if(details_[i].epoch==epoch && details_[i].contactSequence==seq){out=details_[i];return HIGGS_HAND_OK;}
    return HIGGS_HAND_NOT_FOUND;
}
bool ObservationStore::Controllers(const std::array<HIGGS_MotionSample,4>& samples,const HIGGS_TrackingMappingSample* mapping){
    for(unsigned i=0;i<samples.size();++i){const auto& value=samples[i];if(!MotionValid(value)||value.source!=HIGGS_SAMPLE_CONTROLLER||value.physicalHand!=(i/2+1)||value.space!=uint32_t(i%2?HIGGS_SPACE_WORLD:HIGGS_SPACE_OPENVR_TRACKING)||value.epoch!=samples[0].epoch||value.observationId!=samples[0].observationId||value.trackingGeneration!=samples[0].trackingGeneration||value.captureId!=samples[0].captureId||value.observedAtNs!=samples[0].observedAtNs)return false;
        if((value.flags&HIGGS_SAMPLE_MAPPING)&&(!mapping||value.spaceGeneration!=mapping->spaceGeneration))return false;}
    for(unsigned hand=0;hand<2;++hand){const auto& raw=samples[hand*2];const auto& world=samples[hand*2+1];if(raw.deviceIndex!=world.deviceIndex||raw.handGeneration!=world.handGeneration||(world.status==HIGGS_SAMPLE_AVAILABLE&&raw.status!=HIGGS_SAMPLE_AVAILABLE))return false;}
    if(mapping&&(!MappingValid(*mapping)||mapping->epoch!=samples[0].epoch||mapping->trackingGeneration!=samples[0].trackingGeneration||mapping->captureId!=samples[0].captureId||mapping->observedAtNs!=samples[0].observedAtNs))return false;
    std::lock_guard lock(mutex_);EpochLocked(samples[0].epoch);const auto required=uint64_t(4+(mapping?1:0));
    if(samples[0].epoch!=epoch_||required>(std::numeric_limits<uint64_t>::max)()-next_)return false;
    if(samples[0].trackingGeneration<lastTrackingGeneration_||(samples[0].trackingGeneration==lastTrackingGeneration_&&samples[0].captureId<=lastCapture_))return false;
    if(mapping){auto value=*mapping;value.sequence=next_++;Append(mappings_,mappingCount_,mappingWrite_,value);}
    for(auto value:samples){value.sequence=next_++;Append(motion_,motionCount_,motionWrite_,value);}lastTrackingGeneration_=samples[0].trackingGeneration;lastCapture_=samples[0].captureId;return true;
}
void ObservationStore::Release(const HIGGS_HandEvent& event,const HIGGS_HandSnapshot& snapshot){
    if(event.structSize!=sizeof(event)||event.type!=HIGGS_EVENT_RELEASED||!event.eventId||!event.epoch||!event.observationId)return;
    std::lock_guard lock(mutex_);EpochLocked(event.epoch);if(event.epoch!=epoch_)return;
    HIGGS_ReleaseObservation value{};value.structSize=sizeof(value);value.event=event;
    if(snapshot.structSize==sizeof(snapshot)&&snapshot.epoch==event.epoch&&snapshot.observationId==event.observationId&&snapshot.hands[0].physicalHand==HIGGS_HAND_LEFT&&snapshot.hands[1].physicalHand==HIGGS_HAND_RIGHT&&(event.physicalHand==1||event.physicalHand==2)&&!std::memcmp(&snapshot.hands[event.physicalHand-1],&event.hand,sizeof(event.hand))){value.hasMatchingSnapshot=1;value.snapshot=snapshot;}
    for(size_t i=0;i<releaseCount_;++i)if(releases_[i].event.eventId==event.eventId)return;
    Append(releases_,releaseCount_,releaseWrite_,value);
}
HIGGS_HandStatus ObservationStore::CopyMotion(const HIGGS_ObservationQuery& q,HIGGS_MotionSample* out,uint32_t cap,uint32_t& needed){
    if(!Query(q)||(cap&&!out)||cap>motion_.size()||q.source>3||q.space>2)return HIGGS_HAND_INVALID_ARGUMENT;
    std::lock_guard lock(mutex_);if(q.epoch!=epoch_)return HIGGS_HAND_STALE;return Copy(motion_,motionCount_,q,out,cap,needed);
}
HIGGS_HandStatus ObservationStore::CopyContacts(const HIGGS_ObservationQuery& q,HIGGS_ContactObservation* out,uint32_t cap,uint32_t& needed){
    if(!Query(q)||(cap&&!out)||cap>contacts_.size()||q.source>3||(q.space&&q.space!=HIGGS_SPACE_WORLD))return HIGGS_HAND_INVALID_ARGUMENT;
    std::lock_guard lock(mutex_);if(q.epoch!=epoch_)return HIGGS_HAND_STALE;return Copy(contacts_,contactCount_,q,out,cap,needed);
}
HIGGS_HandStatus ObservationStore::CopyMappings(const HIGGS_ObservationQuery& q,HIGGS_TrackingMappingSample* out,uint32_t cap,uint32_t& needed){
    if(!Query(q)||(cap&&!out)||cap>mappings_.size()||(q.source&&q.source!=HIGGS_SAMPLE_CONTROLLER)||(q.space&&q.space!=HIGGS_SPACE_WORLD))return HIGGS_HAND_INVALID_ARGUMENT;
    std::lock_guard lock(mutex_);if(q.epoch!=epoch_)return HIGGS_HAND_STALE;return Copy(mappings_,mappingCount_,q,out,cap,needed);
}
HIGGS_HandStatus ObservationStore::CopyRelease(uint64_t epoch,uint64_t id,HIGGS_ReleaseObservation& out){
    if(!epoch||!id)return HIGGS_HAND_INVALID_ARGUMENT;std::lock_guard lock(mutex_);if(epoch!=epoch_)return HIGGS_HAND_STALE;
    for(size_t i=0;i<releaseCount_;++i)if(releases_[i].event.eventId==id&&releases_[i].event.epoch==epoch){out=releases_[i];return HIGGS_HAND_OK;}return HIGGS_HAND_NOT_FOUND;
}
ObservationStore& Observations(){static ObservationStore value;return value;}
void ObserveRelease(const HIGGS_HandEvent& event,const HIGGS_HandSnapshot& snapshot){Observations().Release(event,snapshot);}
}
