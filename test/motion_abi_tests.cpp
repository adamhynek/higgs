#include "motion_runtime.h"
#include <atomic>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

extern "C" int motion_c_consumer(HIGGS_QueryMotionInterfaceFn);
extern "C" int motion_c_accepts(const HIGGS_MotionSnapshot*, uint32_t, uint64_t);
namespace {
    using namespace HiggsMotion;
    int checks = 0;
    void Check(bool okay) { ++checks; if (!okay) throw std::runtime_error("motion ABI check " + std::to_string(checks)); }
    Event World(std::uintptr_t world) { Event event; event.kind = EventKind::World; event.world = world; return event; }
    Event Sample(ObservationTicket ticket, std::uint64_t now, float value = 3) {
        Event event; event.expectedGeneration = ticket.generation; event.observationSequence = ticket.sequence; event.observedAtNs = now;
        event.hands = {Observation{Status::Valid, 1, {value,0,0}, {0,value,0}}, Observation{Status::Valid, 2, {-value,0,0}, {0,-value,0}}};
        return event;
    }
    CopiedTracking Copied(const State& state, std::uint64_t now) {
        return {{state.hands[0].snapshot,state.hands[1].snapshot},state.generation,state.lastAcceptedObservation,now,state.world!=0,state.loading,state.exhausted};
    }
    HIGGS_MotionInterface Api() { HIGGS_MotionInterface api{}; Check(HIGGS_QueryMotionInterface(1,sizeof(api),&api)==HIGGS_MOTION_OK); return api; }
    HIGGS_MotionSnapshot Read(const HIGGS_MotionInterface& api) { HIGGS_MotionSnapshot out{}; out.structSize=sizeof(out); Check(api.CopyVelocitySnapshot(&out)==HIGGS_MOTION_OK); return out; }
    bool Zero(HIGGS_MotionVector3 value) { return value.x==0&&value.y==0&&value.z==0; }
    void Negotiation() {
        HIGGS_MotionInterface api; std::memset(&api,0xa5,sizeof(api)); const auto sentinel=api;
        for (auto version : {0u,2u,UINT32_MAX}) {
            Check(HIGGS_QueryMotionInterface(version,sizeof(api),&api)==HIGGS_MOTION_UNSUPPORTED_VERSION);
            Check(!std::memcmp(&api,&sentinel,sizeof(api)));
        }
        Check(HIGGS_QueryMotionInterface(1,sizeof(api)-1,&api)==HIGGS_MOTION_BUFFER_TOO_SMALL);
        Check(!std::memcmp(&api,&sentinel,sizeof(api)));
        Check(HIGGS_QueryMotionInterface(1,sizeof(api),nullptr)==HIGGS_MOTION_INVALID_ARGUMENT);
        struct ExtendedTable { HIGGS_MotionInterface table; std::uint64_t tail; } extended{}; extended.tail=UINT64_MAX;
        Check(HIGGS_QueryMotionInterface(1,sizeof(extended),&extended.table)==HIGGS_MOTION_OK&&extended.tail==UINT64_MAX);
        api=extended.table; Check(api.capabilities==15&&api.historySize==5&&api.reserved==0);
        Check(api.CopyVelocitySnapshot(nullptr)==HIGGS_MOTION_INVALID_ARGUMENT);
        HIGGS_MotionSnapshot small; std::memset(&small,0xa5,sizeof(small)); small.structSize=sizeof(small)-1; const auto old=small;
        Check(api.CopyVelocitySnapshot(&small)==HIGGS_MOTION_BUFFER_TOO_SMALL&&!std::memcmp(&small,&old,sizeof(small)));
        struct ExtendedSnapshot { HIGGS_MotionSnapshot state; std::uint64_t tail; } extension{}; extension.state.structSize=sizeof(extension); extension.tail=UINT64_MAX;
        Check(api.CopyVelocitySnapshot(&extension.state)==HIGGS_MOTION_OK&&extension.tail==UINT64_MAX&&extension.state.structSize==232);
    }
    void OwnerAndConsumers() {
        auto api=Api(); auto& owner=Tracking(); owner.Apply({EventKind::NewGame}); auto out=Read(api);
        Check(!out.availableHands&&!out.flags&&out.hands[0].acceptedObservation==0&&out.hands[0].observedAtNs==0&&out.hands[0].validFields==0&&!out.hands[0].historyObservationCount);
        Check(Zero(out.hands[0].latestLinear)&&Zero(out.hands[0].latestAngular)&&Zero(out.hands[0].meanLinear)&&Zero(out.hands[0].peakLinear)&&out.hands[0].meanSpeed==0);
        owner.Apply(World(1)); const Rotation identity{{{1,0,0},{0,1,0},{0,0,1}}};
        for (unsigned i=0;i<5;++i) { auto event=Sample(owner.BeginObservation(),NowNs());
            event.hands={Convert(1,{1,2,3},{4,5,6},identity),Convert(2,{},{},identity)};owner.Apply(event); }
        Check(motion_c_consumer(HIGGS_QueryMotionInterface)==0);
        out=Read(api); const auto copy=owner.CopyObservation();
        Check(copy.generation==out.continuityGeneration&&copy.lastAcceptedObservation==out.processAcceptedObservation&&copy.worldObserved);
        Check(out.hands[0].validFields==63&&out.hands[1].validFields==63&&out.hands[0].ageNs==out.copiedAtNs-out.hands[0].observedAtNs);
        const auto ticket=owner.BeginObservation(); auto invalid=Sample(ticket,NowNs()); invalid.hands[0].status=Status::Disconnected; owner.Apply(invalid);
        out=Read(api); Check(out.availableHands==HIGGS_MOTION_RIGHT&&out.hands[0].status==HIGGS_MOTION_DISCONNECTED&&out.hands[0].validFields==HIGGS_MOTION_AGE_KNOWN);
        Check(out.hands[0].acceptedObservation==ticket.sequence&&Zero(out.hands[0].latestLinear)&&out.hands[0].meanSpeed==0&&!out.hands[0].historyObservationCount);
        Check(!motion_c_accepts(&out,HIGGS_MOTION_LEFT,UINT64_MAX));
        const auto generation=out.continuityGeneration; owner.Apply({EventKind::BeginLoad});out=Read(api);
        Check(out.flags==HIGGS_MOTION_LOADING&&out.continuityGeneration>generation&&!out.availableHands);
        Check(!out.hands[0].acceptedObservation&&!out.hands[1].acceptedObservation&&!out.hands[0].sourceFlags&&!out.hands[0].ageNs&&!out.hands[0].validFields);
        Check(out.processAcceptedObservation==ticket.sequence); // Process watermark is NOT sample presence.
        owner.Apply(Sample(ticket,NowNs())); Check(!Read(api).availableHands);
        owner.Apply({EventKind::EndLoad}); Check(!Read(api).flags);
        owner.Apply(World(2));owner.Apply(Sample(owner.BeginObservation(),0));out=Read(api);
        Check(out.availableHands==3&&out.hands[0].observedAtNs==0&&(out.hands[0].validFields&HIGGS_MOTION_AGE_KNOWN)&&out.hands[0].historyObservationCount==1);
        owner.Apply(World(0));out=Read(api);Check(!out.flags&&!out.availableHands&&!out.hands[0].acceptedObservation);
    }
    void ExactAgesAndBoundaries() {
        const auto max=(std::numeric_limits<std::uint64_t>::max)();
        State state=Next({},World(1));state.lastIssuedObservation=max-1;
        state=Next(state,{EventKind::BeginObservation});state=Next(state,Sample({state.generation,max},max));
        auto out=VelocitySnapshot(Copied(state,max));
        Check(out.availableHands==3&&out.processAcceptedObservation==max&&out.hands[0].acceptedObservation==max&&out.hands[0].ageNs==0&&out.hands[0].validFields==63);
        Check(motion_c_accepts(&out,HIGGS_MOTION_LEFT,0));
        out=VelocitySnapshot(Copied(state,max-1));Check(out.availableHands==3&&out.hands[0].validFields==31&&out.hands[0].ageNs==0);
        Check(!motion_c_accepts(&out,HIGGS_MOTION_LEFT,max)); // Future time is unknown, never zero-aged valid.
        state=Next(state,{EventKind::BeginObservation});out=VelocitySnapshot(Copied(state,max));
        Check(out.flags==HIGGS_MOTION_EXHAUSTED&&!out.availableHands&&out.processAcceptedObservation==max&&!out.hands[0].acceptedObservation&&!out.hands[0].validFields);
        state=Next(state,{EventKind::NewGame});state=Next(state,World(2));state=Next(state,Sample({state.generation,max},0));
        Check(VelocitySnapshot(Copied(state,max)).flags==HIGGS_MOTION_EXHAUSTED);
        state=Next({},World(1));state.generation=max;state=Next(state,World(2));out=VelocitySnapshot(Copied(state,max));
        Check(out.continuityGeneration==max&&out.flags==HIGGS_MOTION_EXHAUSTED&&!out.availableHands);
        state=Next({},World(1));state=Next(state,{EventKind::BeginObservation});state=Next(state,Sample({state.generation,1},0));out=VelocitySnapshot(Copied(state,max));
        Check(out.hands[0].ageNs==max&&out.hands[0].validFields==63&&motion_c_accepts(&out,HIGGS_MOTION_LEFT,max)&&!motion_c_accepts(&out,HIGGS_MOTION_LEFT,max-1));
        for (auto status : {Status::Unavailable,Status::InvalidPose,Status::Disconnected,Status::InvalidConversion,Status::NonFinite,Status::WorldUnavailable}) {
            state=Next(state,{EventKind::BeginObservation});auto event=Sample({state.generation,state.lastIssuedObservation},0);event.hands[0].status=status;state=Next(state,event);
            out=VelocitySnapshot(Copied(state,0));Check(!out.hands[0].meanSpeed&&Zero(out.hands[0].peakLinear)&&out.hands[0].validFields==32&&out.hands[0].status==static_cast<unsigned>(status));
        }
        Check(!motion_c_accepts(&out,0,max)&&!motion_c_accepts(&out,3,max));
    }
    void AtomicReads() {
        auto api=Api();auto& owner=Tracking();owner.Apply({EventKind::NewGame});owner.Apply(World(1));
        std::atomic<bool> done=false,okay=true;
        std::thread writer([&]{for(unsigned i=0;i<10000;++i){if(i%13==0){owner.Apply({EventKind::BeginLoad});owner.Apply({EventKind::EndLoad});owner.Apply(World(1));}
            auto event=Sample(owner.BeginObservation(),NowNs(),float(i%17));if(i%2)event.hands[0].status=Status::InvalidPose;owner.Apply(event);}done=true;});
        const auto reader=[&]{std::uint64_t generation=0;do {HIGGS_MotionSnapshot out{};out.structSize=sizeof(out);
            if(api.CopyVelocitySnapshot(&out)!=HIGGS_MOTION_OK||out.continuityGeneration<generation)okay=false;generation=out.continuityGeneration;
            if(out.hands[0].acceptedObservation!=out.hands[1].acceptedObservation)okay=false;
            if(out.hands[0].acceptedObservation&&out.hands[0].acceptedObservation!=out.processAcceptedObservation)okay=false;
            if((out.flags&HIGGS_MOTION_LOADING)||!(out.flags&HIGGS_MOTION_WORLD_OBSERVED)){if(out.availableHands||out.hands[0].acceptedObservation||out.hands[1].acceptedObservation)okay=false;}
            for(const auto& hand:out.hands){if(!(out.availableHands&hand.physicalHand)&&(!Zero(hand.latestLinear)||!Zero(hand.latestAngular)||hand.meanSpeed!=0))okay=false;
                if((out.availableHands&hand.physicalHand)&&hand.latestLinear.x!=hand.latestAngular.y)okay=false;
                if((out.availableHands&hand.physicalHand)?(hand.historyObservationCount<1||hand.historyObservationCount>5):hand.historyObservationCount!=0)okay=false;
                if((hand.validFields&HIGGS_MOTION_AGE_KNOWN)&&hand.ageNs!=out.copiedAtNs-hand.observedAtNs)okay=false;}
            if(out.availableHands==3&&out.hands[0].latestLinear.x!=-out.hands[1].latestLinear.x)okay=false;
        }while(!done);};
        std::thread left(reader),right(reader);writer.join();left.join();right.join();Check(okay);
    }
    void HistoryCounts() {
        ControllerTracking owner;owner.Apply(World(1));
        for(unsigned i=1;i<=7;++i){owner.Apply(Sample(owner.BeginObservation(),i,0));const auto out=VelocitySnapshot(owner.CopyObservation());
            Check(out.hands[0].historyObservationCount==(i<5?i:5)&&out.hands[1].historyObservationCount==(i<5?i:5));
            Check(out.hands[0].meanSpeed==0&&out.availableHands==3);} // Valid zero observations still fill history.
        auto changed=Sample(owner.BeginObservation(),8);changed.hands[0].device=7;owner.Apply(changed);auto out=VelocitySnapshot(owner.CopyObservation());
        Check(out.hands[0].historyObservationCount==1&&out.hands[1].historyObservationCount==5&&out.hands[0].meanSpeed==0.6f);
        changed=Sample(owner.BeginObservation(),9);changed.hands[0].status=Status::InvalidPose;owner.Apply(changed);out=VelocitySnapshot(owner.CopyObservation());
        Check(out.hands[0].historyObservationCount==0&&out.hands[1].historyObservationCount==5);
        owner.Apply(Sample(owner.BeginObservation(),10));out=VelocitySnapshot(owner.CopyObservation());Check(out.hands[0].historyObservationCount==1);
        owner.Apply({EventKind::BeginLoad});out=VelocitySnapshot(owner.CopyObservation());Check(!out.hands[0].historyObservationCount&&!out.hands[1].historyObservationCount);
    }
}
int main(){try{Negotiation();OwnerAndConsumers();ExactAgesAndBoundaries();AtomicReads();HistoryCounts();std::cout<<"PASS velocity ABI/core/C consumer "<<checks<<" checks\n";return 0;}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
