#include <array>
#include <mutex>
// Forced counter boundaries exercise production refusal, without allocating an
// impossible number of samples. No modified production implementation is linked.
#define private public
#include "hand_observations.h"
#undef private
#include "hand_broker.h"
#include <atomic>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
extern "C" unsigned ObservationCLayout(void);
using namespace HiggsHands;
namespace {
std::atomic<unsigned> checks=0;
#define CHECK(x) do{++checks;if(!(x))throw std::runtime_error(std::string("line ")+std::to_string(__LINE__)+": " #x);}while(false)
HIGGS_MotionSample Sample(uint64_t epoch=1){HIGGS_MotionSample s{};s.structSize=sizeof(s);s.physicalHand=1;s.source=HIGGS_SAMPLE_PALM;s.space=HIGGS_SPACE_WORLD;s.flags=HIGGS_SAMPLE_POSE|HIGGS_SAMPLE_NO_HARDWARE_TIME;s.epoch=epoch;s.observationId=10;s.handGeneration=2;s.observedAtNs=100;s.rotation[0]=s.rotation[4]=s.rotation[8]=1;return s;}
HIGGS_ContactObservation Contact(uint64_t epoch=1){HIGGS_ContactObservation c{};c.structSize=sizeof(c);c.physicalHands=HIGGS_HAND_BOTH;c.source=HIGGS_CONTACT_HAND;c.flags=HIGGS_CONTACT_ENABLED|HIGGS_CONTACT_POINT|HIGGS_CONTACT_NORMAL|HIGGS_CONTACT_LINEAR;c.epoch=epoch;c.observationId=10;c.observedAtNs=100;c.targetToProbeNormal[2]=1;return c;}
HIGGS_ObservationQuery Query(uint64_t epoch=1){return {sizeof(HIGGS_ObservationQuery),HIGGS_HAND_BOTH,0,0,epoch,0,0};}
void NumericAndQueries(){
    ObservationStore s;auto sample=Sample();CHECK(s.Motion(sample));
    for(unsigned n=0;n<25;++n){auto invalid=sample;switch(n%10){case 0:invalid.position[0]=std::numeric_limits<float>::quiet_NaN();break;case 1:invalid.rotation[0]=-1;break;case 2:invalid.flags|=0x80000000;break;case 3:invalid.flags&=~HIGGS_SAMPLE_NO_HARDWARE_TIME;break;case 4:invalid.structSize--;break;case 5:invalid.reserved=1;break;case 6:invalid.linearVelocity[0]=1;break;case 7:invalid.source=HIGGS_SAMPLE_HELD_BODY;break;case 8:invalid.space=HIGGS_SPACE_OPENVR_TRACKING;break;case 9:invalid.deviceIndex=2;break;}CHECK(!s.Motion(invalid));}
    std::array<HIGGS_MotionSample,256> rows{};auto q=Query();uint32_t needed=999;CHECK(s.CopyMotion(q,rows.data(),256,needed)==HIGGS_HAND_OK&&needed==1);const auto first=rows[0].sequence;
    std::memset(rows.data(),0xcc,sizeof(rows));auto keep=rows;needed=999;CHECK(s.CopyMotion(q,rows.data(),0,needed)==HIGGS_HAND_BUFFER_TOO_SMALL&&needed==1&&!std::memcmp(rows.data(),keep.data(),sizeof(rows)));
    auto invalid=q;invalid.structSize++;needed=999;CHECK(s.CopyMotion(invalid,rows.data(),256,needed)==HIGGS_HAND_INVALID_ARGUMENT&&needed==999);invalid=q;invalid.epoch=2;CHECK(s.CopyMotion(invalid,rows.data(),256,needed)==HIGGS_HAND_STALE&&needed==999);invalid=q;invalid.physicalHands=4;CHECK(s.CopyMotion(invalid,rows.data(),256,needed)==HIGGS_HAND_INVALID_ARGUMENT);CHECK(s.CopyMotion(q,nullptr,1,needed)==HIGGS_HAND_INVALID_ARGUMENT);CHECK(s.CopyMotion(q,rows.data(),257,needed)==HIGGS_HAND_INVALID_ARGUMENT);
    q.afterSequence=first;CHECK(s.CopyMotion(q,rows.data(),256,needed)==HIGGS_HAND_OK&&!needed);q=Query();q.minObservedAtNs=101;CHECK(s.CopyMotion(q,rows.data(),256,needed)==HIGGS_HAND_OK&&!needed);q=Query();q.physicalHands=HIGGS_HAND_RIGHT;CHECK(s.CopyMotion(q,rows.data(),256,needed)==HIGGS_HAND_OK&&!needed);
    sample={};sample.structSize=sizeof(sample);sample.physicalHand=2;sample.source=HIGGS_SAMPLE_PALM;sample.space=HIGGS_SPACE_WORLD;sample.flags=HIGGS_SAMPLE_NO_HARDWARE_TIME;sample.status=HIGGS_SAMPLE_INVALID;sample.epoch=1;sample.observedAtNs=101;CHECK(s.Motion(sample));q=Query();CHECK(s.CopyMotion(q,rows.data(),256,needed)==HIGGS_HAND_OK&&needed==2&&rows[1].status==HIGGS_SAMPLE_INVALID);
    for(unsigned i=0;i<300;++i){sample=Sample();sample.position[0]=float(i);sample.observedAtNs=200+i;CHECK(s.Motion(sample));}CHECK(s.CopyMotion(q,rows.data(),256,needed)==HIGGS_HAND_OK&&needed==256&&rows[0].position[0]==44&&rows[255].position[0]==299);for(unsigned i=1;i<needed;++i)CHECK(rows[i].sequence>rows[i-1].sequence);
    const auto last=rows[255].sequence;s.Epoch(2);CHECK(!s.Motion(Sample(1)));q=Query(2);CHECK(s.CopyMotion(q,rows.data(),256,needed)==HIGGS_HAND_OK&&!needed);CHECK(s.Motion(Sample(2)));CHECK(s.CopyMotion(q,rows.data(),256,needed)==HIGGS_HAND_OK&&needed==1&&rows[0].sequence>last);
    s.next_=(std::numeric_limits<uint64_t>::max)();CHECK(!s.Motion(Sample(2)));CHECK(!s.Contact(Contact(2)));
}
void ContactsAndRelease(){
    ObservationStore s;auto c=Contact();CHECK(s.Contact(c));auto invalid=c;invalid.targetToProbeNormal[2]=0;CHECK(!s.Contact(invalid));invalid=c;invalid.flags|=0x80000000;CHECK(!s.Contact(invalid));invalid=c;invalid.flags&=~HIGGS_CONTACT_ENABLED;CHECK(!s.Contact(invalid));invalid=c;invalid.flags&=~HIGGS_CONTACT_NORMAL;CHECK(!s.Contact(invalid));invalid=c;invalid.flags&=~HIGGS_CONTACT_POINT;invalid.position[0]=1;CHECK(!s.Contact(invalid));invalid=c;invalid.flags&=~HIGGS_CONTACT_LINEAR;invalid.probeLinearVelocity[0]=1;CHECK(!s.Contact(invalid));
    for(unsigned i=0;i<520;++i){c.position[0]=float(i);c.observedAtNs=200+i;CHECK(s.Contact(c));}std::array<HIGGS_ContactObservation,512> contacts{};auto q=Query();uint32_t n=0;CHECK(s.CopyContacts(q,contacts.data(),512,n)==HIGGS_HAND_OK&&n==512&&contacts[0].position[0]==8&&contacts[511].position[0]==519);q.space=HIGGS_SPACE_OPENVR_TRACKING;n=999;CHECK(s.CopyContacts(q,contacts.data(),512,n)==HIGGS_HAND_INVALID_ARGUMENT&&n==999);
    HIGGS_HandEvent event{};event.structSize=sizeof(event);event.type=HIGGS_EVENT_RELEASED;event.physicalHand=1;event.eventId=50;event.epoch=1;event.observationId=10;HIGGS_HandSnapshot snapshot{};snapshot.structSize=sizeof(snapshot);snapshot.epoch=1;snapshot.observationId=10;snapshot.hands[0].physicalHand=1;snapshot.hands[1].physicalHand=2;snapshot.hands[1].heldRefId=77;
    event.hand=snapshot.hands[0];s.Release(event,snapshot);HIGGS_ReleaseObservation release{};CHECK(s.CopyRelease(1,50,release)==HIGGS_HAND_OK&&release.hasMatchingSnapshot&&release.snapshot.hands[1].heldRefId==77);snapshot.hands[1].heldRefId=99;s.Release(event,snapshot);CHECK(s.CopyRelease(1,50,release)==HIGGS_HAND_OK&&release.snapshot.hands[1].heldRefId==77);
    event.eventId=51;snapshot.observationId++;s.Release(event,snapshot);CHECK(s.CopyRelease(1,51,release)==HIGGS_HAND_OK&&!release.hasMatchingSnapshot);
    for(unsigned i=0;i<70;++i){event.eventId=100+i;s.Release(event,snapshot);}CHECK(s.CopyRelease(1,100,release)==HIGGS_HAND_NOT_FOUND);CHECK(s.CopyRelease(1,169,release)==HIGGS_HAND_OK);auto untouched=release;CHECK(s.CopyRelease(2,169,release)==HIGGS_HAND_STALE&&!std::memcmp(&release,&untouched,sizeof(release)));CHECK(s.CopyRelease(1,0,release)==HIGGS_HAND_INVALID_ARGUMENT);
}
void ControllerPairs(){
    ObservationStore s;std::array<HIGGS_MotionSample,4> samples{};for(unsigned i=0;i<4;++i){samples[i]=Sample();samples[i].source=HIGGS_SAMPLE_CONTROLLER;samples[i].physicalHand=i/2+1;samples[i].space=i%2?HIGGS_SPACE_WORLD:HIGGS_SPACE_OPENVR_TRACKING;samples[i].flags|=HIGGS_SAMPLE_MAPPING;samples[i].spaceGeneration=10;samples[i].trackingGeneration=3;samples[i].captureId=10;samples[i].deviceIndex=i/2+1;}
    HIGGS_TrackingMappingSample mapping{};mapping.structSize=sizeof(mapping);mapping.flags=HIGGS_SAMPLE_NO_HARDWARE_TIME;mapping.epoch=1;mapping.spaceGeneration=10;mapping.trackingGeneration=3;mapping.captureId=10;mapping.observedAtNs=100;mapping.rotation[0]=mapping.rotation[4]=mapping.rotation[8]=1;mapping.unitsPerMeter=70;CHECK(s.Controllers(samples,&mapping));CHECK(!s.Controllers(samples,&mapping));CHECK(!s.Motion(samples[0]));
    std::array<HIGGS_MotionSample,256> motion{};std::array<HIGGS_TrackingMappingSample,64> maps{};auto q=Query();uint32_t n=0;CHECK(s.CopyMotion(q,motion.data(),256,n)==HIGGS_HAND_OK&&n==4);CHECK(s.CopyMappings(q,maps.data(),64,n)==HIGGS_HAND_OK&&n==1&&maps[0].spaceGeneration==motion[0].spaceGeneration);
    auto bad=samples;bad[3].rotation[0]=std::numeric_limits<float>::infinity();CHECK(!s.Controllers(bad,&mapping));CHECK(s.CopyMotion(q,motion.data(),256,n)==HIGGS_HAND_OK&&n==4);bad=samples;bad[3].epoch=2;CHECK(!s.Controllers(bad,&mapping));CHECK(s.CopyMotion(q,motion.data(),256,n)==HIGGS_HAND_OK&&n==4);bad=samples;bad[3].observationId++;CHECK(!s.Controllers(bad,&mapping));bad=samples;bad[1].deviceIndex++;CHECK(!s.Controllers(bad,&mapping));bad=samples;bad[1].flags&=~HIGGS_SAMPLE_MAPPING;bad[1].spaceGeneration=0;CHECK(!s.Controllers(bad,nullptr));
    for(auto& sample:samples){sample.captureId=11;sample.spaceGeneration=11;}mapping.captureId=mapping.spaceGeneration=11;CHECK(s.Controllers(samples,&mapping));mapping.rotation[0]=-1;for(auto& sample:samples){sample.captureId=12;sample.spaceGeneration=12;}mapping.captureId=mapping.spaceGeneration=12;CHECK(!s.Controllers(samples,&mapping));
    s.next_=(std::numeric_limits<uint64_t>::max)()-4;mapping.rotation[0]=1;CHECK(!s.Controllers(samples,&mapping));CHECK(s.CopyMotion(q,motion.data(),256,n)==HIGGS_HAND_OK&&n==8);
    q.space=HIGGS_SPACE_OPENVR_TRACKING;n=999;CHECK(s.CopyMappings(q,maps.data(),64,n)==HIGGS_HAND_INVALID_ARGUMENT&&n==999);
}
HIGGS_HandEvent captured{};
void HIGGS_HAND_CALL Event(const HIGGS_HandEvent* event,void*){if(event->type==HIGGS_EVENT_RELEASED)captured=*event;}
void PublicBoundary(){
    CHECK(ObservationCLayout()==offsetof(HIGGS_MotionSample,position));HIGGS_ObservationAPI api{};CHECK(HIGGS_QueryObservationInterface(2,sizeof(api),&api)==HIGGS_HAND_OK&&api.capabilities==15);auto preserved=api;CHECK(HIGGS_QueryObservationInterface(1,sizeof(api),&api)==HIGGS_HAND_UNSUPPORTED_VERSION&&!std::memcmp(&api,&preserved,sizeof(api)));CHECK(HIGGS_QueryObservationInterface(2,sizeof(api)-1,&api)==HIGGS_HAND_BUFFER_TOO_SMALL);CHECK(HIGGS_QueryObservationInterface(2,sizeof(api),nullptr)==HIGGS_HAND_INVALID_ARGUMENT);
    HIGGS_HandClientInfo info{};info.structSize=sizeof(info);std::memcpy(info.name,"observation-tests",18);HIGGS_HandClient client=0;CHECK(Service().Register(info,client)==HIGGS_HAND_OK);const auto now=Now();Service().ObserveWorld(123,false,now);std::array<HIGGS_HandRecord,2> hands{};for(unsigned i=0;i<2;++i){hands[i].physicalHand=i+1;hands[i].observationMask=HIGGS_OBS_HELD_KNOWN|HIGGS_OBS_EQUIPMENT_KNOWN|HIGGS_OBS_TRACKING_VALID|HIGGS_OBS_PALM_VALID;hands[i].palmRotation[0]=hands[i].palmRotation[4]=hands[i].palmRotation[8]=1;}hands[0].heldRefId=42;hands[1].heldRefId=77;Service().Publish(hands,now+1);uint64_t subscription=0;CHECK(Service().Events(client,Event,nullptr,subscription)==HIGGS_HAND_OK);hands[0].heldRefId=0;hands[1].heldRefId=99;Service().Publish(hands,now+2);CHECK(captured.eventId!=0);HIGGS_ReleaseObservation release{};release.structSize=sizeof(release);CHECK(api.CopyReleaseObservation(client,captured.eventId,&release)==HIGGS_HAND_OK&&release.hasMatchingSnapshot&&release.snapshot.hands[0].heldRefId==0&&release.snapshot.hands[1].heldRefId==99);
    auto q=Query(Service().Epoch());uint32_t n=999;std::array<HIGGS_MotionSample,256> rows{};CHECK(api.CopyMotionSamples(UINT64_MAX,&q,rows.data(),256,&n)==HIGGS_HAND_INVALID_CLIENT&&n==999);auto sample=Sample(q.epoch);CHECK(Observations().Motion(sample));CHECK(api.CopyMotionSamples(client,&q,rows.data(),256,&n)==HIGGS_HAND_OK&&n==1);q.epoch--;n=999;CHECK(api.CopyMotionSamples(client,&q,rows.data(),256,&n)==HIGGS_HAND_STALE&&n==999);CHECK(Service().Unregister(client)==HIGGS_HAND_OK);q.epoch=Service().Epoch();CHECK(api.CopyMotionSamples(client,&q,rows.data(),256,&n)==HIGGS_HAND_INVALID_CLIENT);
}
}
int main(){try{NumericAndQueries();ContactsAndRelease();ControllerPairs();PublicBoundary();std::cout<<"PASS observations "<<checks<<" checks\n";return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
