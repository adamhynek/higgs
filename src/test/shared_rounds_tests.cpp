#include "hand_broker.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
using namespace HiggsHands;
static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x); std::exit(1); } } while (0)
struct Fixture {
    Broker broker;
    uint64_t button=0,dwell=0,buttonExecutor=0,dwellExecutor=0,recognizer=0,lastButton=0,lastDwell=0;
    unsigned buttonEffects=0,dwellEffects=0;
    bool endOnNotify=false,reenter=false;
    uint64_t queued=0;
    std::array<HIGGS_HandRecord,2> hands{};
    Fixture(bool shared=true) {
        HIGGS_HandClientInfo c{}; c.structSize=sizeof(c);
        std::strcpy(c.name,"button"); CHECK(!broker.Register(c,button));
        std::strcpy(c.name,"dwell"); CHECK(!broker.Register(c,dwell));
        CHECK(!broker.Executor(button,ExecuteButton,this,buttonExecutor));
        CHECK(!broker.Executor(dwell,ExecuteDwell,this,dwellExecutor));
        CHECK(!broker.Recognizer(button,Recognize,this,recognizer));
        broker.ObserveWorld(1,false,Now());CHECK(!broker.IsMainThread());broker.MarkMainThread();CHECK(broker.IsMainThread());
        CHECK(broker.ObservedWorldMatches(1));CHECK(!broker.ObservedWorldMatches(2));
        for (auto &h:hands) h.observationMask=HIGGS_OBS_HELD_KNOWN|HIGGS_OBS_EQUIPMENT_KNOWN|
            HIGGS_OBS_PALM_VALID|HIGGS_OBS_TRACKING_VALID;
        Publish();
        if(shared) CHECK(!broker.EnableSharedRounds(dwell));
    }
    void Publish(bool native=false,uint32_t held=0) {
        hands[0].heldRefId=held;
        hands[0].observationMask&=~(HIGGS_OBS_NATIVE_GRIP_TARGET|HIGGS_OBS_NATIVE_TRIGGER_TARGET);
        if(native) hands[0].observationMask|=HIGGS_OBS_NATIVE_GRIP_TARGET;
        broker.SetNativeTargetKeys(HIGGS_HAND_LEFT,native?123:0,0);
        broker.Publish(hands,Now());
    }
    static void HIGGS_HAND_CALL Recognize(const HIGGS_HandInput *input,void *context) {
        auto &f=*static_cast<Fixture*>(context);
        if(input->edge!=HIGGS_INPUT_DOWN)return;
        HIGGS_ActionCandidate c{sizeof(c),HIGGS_HAND_LEFT,input->inputId,input->epoch,input->handGeneration,
            f.buttonExecutor,100,88,1,1000,0};
        CHECK(!f.broker.Submit(f.button,c,f.lastButton,Now(),55));
    }
    static void HIGGS_HAND_CALL ExecuteButton(const HIGGS_ActionRequest *r,void *context) {
        auto &f=*static_cast<Fixture*>(context);++f.buttonEffects;
        if(f.endOnNotify)CHECK(!f.broker.Release(f.button,r->grant));
    }
    static void HIGGS_HAND_CALL ExecuteDwell(const HIGGS_ActionRequest *r,void *context) {
        auto &f=*static_cast<Fixture*>(context);++f.dwellEffects;
        if(f.reenter) {
            f.reenter=false; f.queued=f.Propose(HIGGS_HAND_RIGHT,901,71);
            const auto before=f.dwellEffects;f.broker.DrainHandUses(); CHECK(f.dwellEffects==before);
        }
        if(f.endOnNotify)CHECK(!f.broker.Release(f.dwell,r->grant));
    }
    HIGGS_HandUseProposalRequest Proposal(uint32_t handsMask=HIGGS_HAND_LEFT,uint64_t context=70) {
        const auto snapshot=broker.CopyPublished();
        HIGGS_HandUseProposalRequest p{};p.structSize=sizeof(p);p.deadlineMs=1000;
        for(unsigned i=0;i<2;++i)p.expectedHandGenerations[i]=snapshot.hands[i].generation;
        p.use={sizeof(p.use),HIGGS_USE_CONSUMPTION,handsMask,1000,snapshot.epoch,context,1,dwellExecutor};
        return p;
    }
    uint64_t Propose(uint32_t handsMask=HIGGS_HAND_LEFT,uint64_t action=900,uint64_t target=70) {
        auto p=Proposal(handsMask,target); uint64_t id=0;
        CHECK(!broker.ProposeOrderedHandUse(dwell,p,action,target,id,Now()));lastDwell=id;return id;
    }
    HIGGS_HandUseProposalState State(uint64_t id) {
        HIGGS_HandUseProposalState s{};CHECK(!broker.CopyHandUseProposal(dwell,id,s,Now()));return s;
    }
};
void MixedOrders() {
    for(unsigned winner=0;winner<3;++winner)for(bool dwellFirst:{false,true}) {
        auto f=std::make_unique<Fixture>();f->Publish(true);f->endOnNotify=true;
        CHECK(f->broker.ConfigurePriorities(winner==0?"button/100=30,dwell/900=20,higgs/1=10":
            winner==1?"button/100=20,dwell/900=30,higgs/1=10":"button/100=20,dwell/900=10,higgs/1=30"));
        uint64_t proposal=0;if(dwellFirst)proposal=f->Propose();
        bool native=false;CHECK(f->broker.Input(1,HIGGS_INPUT_GRIP,true,Now(),&native));CHECK(!native);
        if(!dwellFirst)proposal=f->Propose();
        CHECK(!f->buttonEffects&&!f->dwellEffects);
        f->broker.DrainHandUses();
        CHECK(f->buttonEffects==(winner==0));CHECK(f->dwellEffects==(winner==1));
        CHECK(f->State(proposal).stage==(winner==1?HIGGS_USE_PROPOSAL_ACCEPTED:HIGGS_USE_PROPOSAL_REJECTED));
        const bool blocked=f->broker.Input(1,HIGGS_INPUT_GRIP,true,Now(),&native);
        CHECK(native==(winner==2));CHECK(blocked==(winner!=2));
        f->broker.DrainHandUses();CHECK(f->buttonEffects+f->dwellEffects==(winner==2?0u:1u));
        f->broker.Input(1,HIGGS_INPUT_GRIP,false,Now(),&native);CHECK(native==(winner==2));
    }
}
void CancelAndNoReplay() {
    for(unsigned invalidation=0;invalidation<4;++invalidation) {
        auto f=std::make_unique<Fixture>();f->Publish(true);
        CHECK(f->broker.ConfigurePriorities("higgs/1=100,button/100=1"));
        bool native=false;CHECK(f->broker.Input(1,HIGGS_INPUT_GRIP,true,Now(),&native));
        if(invalidation==0) CHECK(f->broker.Input(1,HIGGS_INPUT_GRIP,false,Now(),&native));
        if(invalidation==1){f->broker.ObserveWorld(0,true,Now());CHECK(!f->broker.ObservedWorldMatches(1));}
        if(invalidation==2)f->broker.SetInputContext(1,Now());
        if(invalidation==3)f->broker.SetNativeTargetKeys(1,456,0);
        f->broker.DrainHandUses();CHECK(f->buttonEffects==(invalidation==3));
        if(invalidation==0||invalidation==3) {
            f->broker.Input(1,HIGGS_INPUT_GRIP,true,Now(),&native);CHECK(!native);
        }
    }
    auto f=std::make_unique<Fixture>();f->Publish(true);
    CHECK(f->broker.ConfigurePriorities("higgs/1=100"));
    bool native=false;CHECK(f->broker.Input(1,1,true,Now(),&native));f->broker.DrainHandUses();
    CHECK(f->broker.Input(1,1,false,Now(),&native));CHECK(!native); // selected, never delivered tap
}
void NativePossessionAndHandoff() {
    auto f=std::make_unique<Fixture>();f->Publish(true);
    CHECK(f->broker.ConfigurePriorities("higgs/1=100,button/100=1,dwell/900=200"));
    bool native=false;CHECK(f->broker.Input(1,1,true,Now(),&native));f->broker.DrainHandUses();
    CHECK(!f->broker.Input(1,1,true,Now(),&native)&&native);
    auto p=f->Proposal();uint64_t rejected=0;
    CHECK(f->broker.ProposeOrderedHandUse(f->dwell,p,900,70,rejected,Now())==HIGGS_HAND_CONFLICT);
    f->Publish(false,99);const auto id=f->Propose();f->broker.DrainHandUses();
    CHECK(f->dwellEffects==1&&f->State(id).use.active); // grip remains down; held consumption is valid
    CHECK(!f->broker.Input(1,1,true,Now(),&native)&&native);
    CHECK(!f->broker.EndHandUse(f->dwell,id,f->broker.Epoch()));
    f->Publish(false,0); // consumed/transferred item is gone while the original grip remains down
    auto again=f->Propose();f->broker.DrainHandUses();CHECK(f->dwellEffects==2&&f->State(again).use.active);
    CHECK(!f->broker.Input(1,1,false,Now(),&native)&&native);
}
void NativeRegions() {
    for(bool nativeWins:{false,true})for(bool regionFirst:{false,true}) {
        auto f=std::make_unique<Fixture>();f->Publish(false,99);
        CHECK(f->broker.ConfigurePriorities(nativeWins?"higgs/2=100,dwell/900=1":"higgs/2=1,dwell/900=100"));
        if(regionFirst)CHECK(f->broker.NativeRegionAdmission(1,f->broker.Epoch(),99,2,Now())==HIGGS_HAND_PENDING);
        const auto id=f->Propose();
        if(!regionFirst)CHECK(f->broker.NativeRegionAdmission(1,f->broker.Epoch(),99,2,Now())==HIGGS_HAND_PENDING);
        f->broker.DrainHandUses();CHECK(f->dwellEffects==!nativeWins);
        CHECK(f->broker.NativeRegionAdmission(1,f->broker.Epoch(),99,2,Now())==
              (nativeWins?HIGGS_HAND_OK:HIGGS_HAND_CONFLICT));
        if(nativeWins) {
            f->broker.FinishNativeRegionAction(1,true);
            CHECK(f->broker.NativeRegionAdmission(1,f->broker.Epoch(),99,2,Now())==HIGGS_HAND_CONFLICT);
        } else CHECK(f->State(id).use.active);
    }
}
void KeysAndReentry() {
    auto f=std::make_unique<Fixture>();
    const auto a=f->Propose(1,900,30),b=f->Propose(1,900,20);
    f->broker.DrainHandUses();CHECK(f->State(a).stage==HIGGS_USE_PROPOSAL_REJECTED);
    CHECK(f->State(b).stage==HIGGS_USE_PROPOSAL_ACCEPTED);
    CHECK(!f->broker.EndHandUse(f->dwell,b,f->broker.Epoch()));
    f->reenter=true;const auto c=f->Propose(1,901,40);f->broker.DrainHandUses();
    CHECK(f->queued&&f->dwellEffects==2&&f->State(f->queued).stage==HIGGS_USE_PROPOSAL_PENDING);
    f->broker.DrainHandUses();CHECK(f->dwellEffects==3&&f->State(f->queued).use.active);(void)c;
}
void LegacyAndAbi() {
    auto f=std::make_unique<Fixture>(false);CHECK(f->broker.AcceptNativePress(1,1));f->Publish(true);
    CHECK(f->broker.ConfigurePriorities("higgs/1=100"));bool native=false;
    CHECK(!f->broker.Input(1,1,true,Now(),&native)&&native); // unchanged synchronous native default
    f->broker.Input(1,1,false,Now(),&native);
    CHECK(!f->broker.EnableSharedRounds(f->dwell));
    CHECK(!f->broker.AcceptNativePress(1,1));
    auto p=f->Proposal();HIGGS_HandUseState state{};
    CHECK(f->broker.HandUse(f->dwell,p.use,state,Now())==HIGGS_HAND_WRONG_PHASE);
    HIGGS_ActionRequest authority{};
    CHECK(f->broker.ReserveDomain(f->dwell,0,1,f->broker.Epoch(),0,1000,authority,Now())==HIGGS_HAND_WRONG_PHASE);
    HIGGS_HandUseProposalsInterface old{};CHECK(!HIGGS_QueryHandUseProposalsInterface(1,sizeof(old),&old));
    CHECK(old.structSize==sizeof(old)&&old.abiVersion==1&&old.capabilities==1);
    HIGGS_SharedHandUseProposalsInterface shared{};
    CHECK(!HIGGS_QueryHandUseProposalsInterface(2,sizeof(shared),&shared.proposals));
    CHECK(shared.proposals.abiVersion==2&&shared.EnableSharedRounds&&shared.ProposeOrdered);
    CHECK(HIGGS_QueryHandUseProposalsInterface(2,sizeof(old),&old)==HIGGS_HAND_BUFFER_TOO_SMALL);
}
void ResizeAndNativeLifecycle() {
    for(bool expansionWins:{false,true}) {
        auto f=std::make_unique<Fixture>();
        const auto first=f->Propose(1,900,10);f->broker.DrainHandUses();const auto parent=f->State(first).use;
        const auto snapshot=f->broker.CopyPublished();
        HIGGS_HandUseTransitionRequest r{sizeof(r),3,1000,0,parent.epoch,parent.authority.requestId,1,
            {snapshot.hands[0].generation,snapshot.hands[1].generation}};
        CHECK(f->broker.ConfigurePriorities(expansionWins?"dwell/999=100,dwell/900=1":"dwell/999=1,dwell/900=100"));
        uint64_t resize=0;CHECK(!f->broker.ProposeHandUseResize(f->dwell,r,999,10,resize,Now()));
        const auto rival=f->Propose(2,900,20);f->broker.DrainHandUses();
        CHECK(f->State(resize).stage==(expansionWins?HIGGS_USE_PROPOSAL_ACCEPTED:HIGGS_USE_PROPOSAL_REJECTED));
        HIGGS_HandUseTransitionState state{};CHECK(!f->broker.CopyHandUseTransition(f->dwell,parent.authority.requestId,state,Now()));
        CHECK(state.use.active&&state.use.authority.grant==parent.authority.grant);
        CHECK(state.use.authority.physicalHands==(expansionWins?3u:1u));
        CHECK(f->State(rival).stage==(expansionWins?HIGGS_USE_PROPOSAL_REJECTED:HIGGS_USE_PROPOSAL_ACCEPTED));
    }
    auto f=std::make_unique<Fixture>();CHECK(!f->broker.Remove(f->button,f->recognizer));
    f->Publish(true);f->hands[0].observationMask|=HIGGS_OBS_NATIVE_TRIGGER_TARGET;
    f->broker.SetNativeTargetKeys(1,123,123);f->broker.Publish(f->hands,Now());
    bool native=false;CHECK(f->broker.Input(1,1,true,Now(),&native));CHECK(f->broker.Input(1,2,true,Now(),&native));
    f->broker.DrainHandUses();CHECK(!f->broker.Input(1,1,true,Now(),&native)&&native);
    CHECK(!f->broker.Input(1,2,true,Now(),&native)&&native);
    CHECK(!f->broker.Input(1,1,false,Now()+UINT64_C(2000000000),&native)&&native);
    CHECK(!f->broker.Input(1,2,false,Now()+UINT64_C(2000000000),&native)&&native);
    for(unsigned n=0;n<270;++n) {
        f->Publish(true);CHECK(f->broker.Input(1,1,true,Now(),&native));f->broker.DrainHandUses();
        CHECK(!f->broker.Input(1,1,true,Now(),&native)&&native);
        CHECK(!f->broker.Input(1,1,false,Now(),&native)&&native);
    }
    f->Publish(false);CHECK(!f->broker.Input(1,HIGGS_INPUT_TRIGGER,true,Now(),&native)&&!native);
    CHECK(!f->broker.AcceptNativePress(1,HIGGS_INPUT_TRIGGER)); // vanilla flows, HIGGS leeway cannot start
    CHECK(!f->broker.Input(1,HIGGS_INPUT_TRIGGER,false,Now(),&native));
    CHECK(!f->broker.Input(1,kAuxiliaryInputBase+7,true,Now(),&native));
}
struct Extended {
    uint64_t owner=0,executor=0,recognizer=0,request=0;
    unsigned effects=0;
    static void HIGGS_HAND_CALL Execute(const HIGGS_ActionRequest *,void *p) { ++static_cast<Extended*>(p)->effects; }
    static void HIGGS_HAND_CALL Recognize(const HIGGS_InputEvent *e,void *p) {
        if (!(e->flags&HIGGS_INPUT_ACTIONABLE)||e->kind!=HIGGS_RAW_PRESS)return;
        auto &x=*static_cast<Extended*>(p);HIGGS_InputCandidate c{};c.structSize=sizeof(c);
        c.gesture=HIGGS_GESTURE_HOLD;c.defaultPolicy=HIGGS_DEFAULT_DISCARD_AFTER_SELECTION;
        c.deadlineMs=200;c.holdMs=50;
        c.candidate={sizeof(c.candidate),1,e->inputId,e->snapshot.epoch,e->snapshot.hands[0].generation,
                     x.executor,22,0,0,1000,0};
        CHECK(!Service().SubmitInputCandidate(x.owner,c,x.request,e->frame.observedAtNs));
    }
};
void ExtendedLifecycle() {
    for(bool remove:{false,true}) {
        auto &b=Service();b.ObserveWorld(remove?902:901,false,Now());b.MarkMainThread();
        std::array<HIGGS_HandRecord,2> hands{};
        for(auto &h:hands)h.observationMask=HIGGS_OBS_HELD_KNOWN|HIGGS_OBS_EQUIPMENT_KNOWN|
            HIGGS_OBS_PALM_VALID|HIGGS_OBS_TRACKING_VALID;
        b.Publish(hands,Now());Extended x;HIGGS_HandClientInfo info{};info.structSize=sizeof(info);
        std::strcpy(info.name,"extended-shared");CHECK(!b.Register(info,x.owner));
        CHECK(!b.Executor(x.owner,Extended::Execute,&x,x.executor));
        CHECK(!b.EnableSharedRounds(x.owner));
        HIGGS_InputRecognizerSpec spec{};spec.structSize=sizeof(spec);spec.physicalHands=1;
        spec.controlMask=UINT64_C(1)<<HIGGS_CONTROL_TRIGGER;spec.callback=Extended::Recognize;spec.context=&x;
        CHECK(!b.InputRecognizer(x.owner,spec,x.recognizer));
        const auto at=Now();
        const auto frame=[&](bool pressed,uint64_t time) {
            HIGGS_ControllerFrame raw{};raw.structSize=sizeof(raw);raw.physicalHand=1;raw.deviceIndex=1;
            raw.flags=HIGGS_INPUT_NO_HARDWARE_TIME;raw.epoch=b.Epoch();raw.observedAtNs=time;
            raw.handObservationId=b.CopyPublished().observationId;
            raw.pressed=pressed ? UINT64_C(1)<<HIGGS_CONTROL_TRIGGER:0;
            BeginControllerFrame(raw);b.Input(1,HIGGS_INPUT_TRIGGER,pressed,time);
            uint64_t bits=raw.pressed,touch=0;FinishControllerFrame(1,bits,touch);
        };
        frame(false,at);frame(true,at+1000000);CHECK(x.request&&!x.effects);
        if(remove)CHECK(!b.Remove(x.owner,x.recognizer));
        b.DrainHandUses();CHECK(!x.effects);
        frame(true,at+61000000);CHECK(x.effects==(remove?0u:1u));
        b.DrainHandUses();b.DrainHandUses();CHECK(x.effects==(remove?0u:1u));
        frame(false,at+62000000);CHECK(!b.Unregister(x.owner));
    }
}
int main() {
    MixedOrders();CancelAndNoReplay();NativePossessionAndHandoff();NativeRegions();KeysAndReentry();LegacyAndAbi();ResizeAndNativeLifecycle();ExtendedLifecycle();
    std::printf("HIGGS production shared rounds: %u checks passed; zero game starts\n",checks);
}
