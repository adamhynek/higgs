// Included after the authority Fixture: tests exercise the actual broker.
HIGGS_HandUseProposalRequest Proposal(Fixture &f,uint64_t owner,uint64_t executor,uint32_t hands,uint64_t context=1)
{
    HIGGS_HandUseProposalRequest r{};
    r.structSize=sizeof(r); r.deadlineMs=1000;
    r.use={sizeof(r.use),HIGGS_USE_WORLD_SUPPORT,hands,1000,f.broker.Epoch(),context,1,executor};
    HIGGS_HandSnapshot snapshot{};
    CHECK(!f.broker.Copy(owner,snapshot,f.now));
    for (unsigned i=0;i<2;++i) r.expectedHandGenerations[i]=snapshot.hands[i].generation;
    return r;
}
HIGGS_HandUseProposalState ProposalState(Fixture &f,uint64_t owner,uint64_t id)
{
    HIGGS_HandUseProposalState state{};
    CHECK(!f.broker.CopyHandUseProposal(owner,id,state,Now()));
    return state;
}
void ProposalPriorityRounds()
{
    for (unsigned reversed=0;reversed<2;++reversed) {
        auto f=std::make_unique<Fixture>(); uint64_t otherExecutor=0,first=0,second=0;
        CHECK(!f->broker.Executor(f->other,Fixture::Notify,f.get(),otherExecutor));
        auto high=Proposal(*f,f->owner,f->executor,HIGGS_HAND_BOTH);
        auto low=Proposal(*f,f->other,otherExecutor,HIGGS_HAND_LEFT);
        if (reversed) {
            CHECK(!f->broker.ProposeHandUse(f->owner,high,first,Now()));
            CHECK(!f->broker.ProposeHandUse(f->other,low,second,Now()));
        } else {
            CHECK(!f->broker.ProposeHandUse(f->other,low,second,Now()));
            CHECK(!f->broker.ProposeHandUse(f->owner,high,first,Now()));
        }
        auto pending=ProposalState(*f,f->owner,first);
        CHECK(pending.stage==HIGGS_USE_PROPOSAL_PENDING && !pending.use.authority.grant);
        CHECK(!f->notifications);
        f->broker.DrainHandUses();
        auto accepted=ProposalState(*f,f->owner,first),rejected=ProposalState(*f,f->other,second);
        CHECK(accepted.stage==HIGGS_USE_PROPOSAL_ACCEPTED && accepted.use.active);
        CHECK(rejected.stage==HIGGS_USE_PROPOSAL_REJECTED && rejected.reason==HIGGS_HAND_CONFLICT);
        CHECK(accepted.roundId && accepted.roundId==rejected.roundId && accepted.effectivePriority==10);
        CHECK(f->notifications==1 && f->notification.inputId==0);
        CHECK(f->notification.contextId==1 && f->notification.contextGeneration==1);
        CHECK(f->notification.requestId==first && f->notification.grant==accepted.use.authority.grant);
        f->broker.DrainHandUses(); CHECK(f->notifications==1);
        CHECK(f->broker.CancelHandUseProposal(f->owner,first,f->broker.Epoch())==HIGGS_HAND_CONFLICT);
        CHECK(!f->broker.EndHandUse(f->owner,first,f->broker.Epoch()));
        accepted=ProposalState(*f,f->owner,first);
        CHECK(accepted.stage==HIGGS_USE_PROPOSAL_ACCEPTED && !accepted.use.active);
    }
    // User priority changes collected proposals; lower-priority registration wins.
    auto f=std::make_unique<Fixture>(); uint64_t exec=0,one=0,two=0;
    CHECK(!f->broker.Executor(f->other,Fixture::Notify,f.get(),exec));
    CHECK(!f->broker.ProposeHandUse(f->owner,Proposal(*f,f->owner,f->executor,1),one,Now()));
    CHECK(!f->broker.ProposeHandUse(f->other,Proposal(*f,f->other,exec,1),two,Now()));
    f->broker.ConfigurePriorities("primary=-10,other=50");
    f->broker.DrainHandUses();
    auto winner=ProposalState(*f,f->other,two);
    CHECK(winner.use.active && winner.effectivePriority==50 && (winner.decisionFlags&HIGGS_DECISION_USER_PRIORITY));
    CHECK(ProposalState(*f,f->owner,one).stage==HIGGS_USE_PROPOSAL_REJECTED);
    // With equal priority, the stable ID 'other' precedes 'primary' even though it registered later.
    f=std::make_unique<Fixture>();
    CHECK(!f->broker.Executor(f->other,Fixture::Notify,f.get(),exec));
    f->broker.ConfigurePriorities("primary=0,other=0");
    CHECK(!f->broker.ProposeHandUse(f->other,Proposal(*f,f->other,exec,1),two,Now()));
    CHECK(!f->broker.ProposeHandUse(f->owner,Proposal(*f,f->owner,f->executor,1),one,Now()));
    f->broker.DrainHandUses(); CHECK(ProposalState(*f,f->other,two).use.active);
    // Atomic Both rejection leaves a disjoint lower-priority right proposal usable.
    f=std::make_unique<Fixture>();
    CHECK(!f->broker.Executor(f->other,Fixture::Notify,f.get(),exec));
    uint64_t right=0;
    CHECK(!f->broker.ProposeHandUse(f->owner,Proposal(*f,f->owner,f->executor,1),one,Now()));
    CHECK(!f->broker.ProposeHandUse(f->other,Proposal(*f,f->other,exec,3),two,Now()));
    CHECK(!f->broker.ProposeHandUse(f->other,Proposal(*f,f->other,exec,2,2),right,Now()));
    f->broker.DrainHandUses();
    CHECK(ProposalState(*f,f->owner,one).use.active && ProposalState(*f,f->other,right).use.active);
    CHECK(ProposalState(*f,f->other,two).stage==HIGGS_USE_PROPOSAL_REJECTED && f->notifications==2);
}
void ProposalLifetimes()
{
    for (unsigned mode=0;mode<7;++mode) {
        auto f=std::make_unique<Fixture>(); uint64_t id=0;
        auto r=Proposal(*f,f->owner,f->executor,1);
        CHECK(!f->broker.ProposeHandUse(f->owner,r,id,Now()));
        if (mode==0) CHECK(!f->broker.CancelHandUseProposal(f->owner,id,r.use.epoch));
        if (mode==1) { f->broker.SetInputContext(1,Now()); f->broker.SetInputContext(0,Now()); }
        if (mode==2) f->broker.ObserveWorld(2,false,Now());
        if (mode==3) CHECK(!f->broker.Remove(f->owner,f->executor));
        if (mode==4) { f->hands[0].heldRefId=42; f->broker.Publish(f->hands,Now()); }
        if (mode==5) {
            HIGGS_HandUseProposalState s{};
            CHECK(!f->broker.CopyHandUseProposal(f->owner,id,s,Now()+UINT64_C(2000000000)));
        }
        if (mode==6) { f->hands[0].observationMask=0; f->broker.Publish(f->hands,Now()); }
        f->broker.DrainHandUses();
        const auto s=ProposalState(*f,f->owner,id);
        CHECK(s.stage>=HIGGS_USE_PROPOSAL_REJECTED && !s.use.authority.grant && !f->notifications);
    }
    auto f=std::make_unique<Fixture>(); uint64_t id=0,otherId=0;
    auto r=Proposal(*f,f->owner,f->executor,1);
    CHECK(!f->broker.ProposeHandUse(f->owner,r,id,Now()));
    CHECK(f->broker.ProposeHandUse(f->owner,r,otherId,Now())==HIGGS_HAND_CONFLICT);
    HIGGS_HandUseState use{};
    CHECK(f->broker.HandUse(f->owner,r.use,use,Now())==HIGGS_HAND_CONFLICT);
    CHECK(f->broker.CancelHandUseProposal(f->other,id,r.use.epoch)==HIGGS_HAND_WRONG_OWNER);
    CHECK(!f->broker.Unregister(f->owner)); f->broker.DrainHandUses(); CHECK(!f->notifications);
    // Pending per-client bound, then cancellation frees exactly one queue position.
    f=std::make_unique<Fixture>();
    for (unsigned i=0;i<8;++i) CHECK(!f->broker.ProposeHandUse(f->owner,Proposal(*f,f->owner,f->executor,1,i),id,Now()));
    CHECK(f->broker.ProposeHandUse(f->owner,Proposal(*f,f->owner,f->executor,1,99),otherId,Now())==HIGGS_HAND_CAPACITY);
    CHECK(!f->broker.CancelHandUseProposal(f->owner,id,f->broker.Epoch()));
    CHECK(!f->broker.ProposeHandUse(f->owner,Proposal(*f,f->owner,f->executor,1,99),otherId,Now()));
}
struct ProposalReentry {
    Fixture *fixture;
    uint64_t executor=0,queued=0;
    unsigned calls=0;
    static void HIGGS_HAND_CALL Notify(const HIGGS_ActionRequest *,void *p) {
        auto &self=*static_cast<ProposalReentry *>(p); ++self.calls;
        if (self.calls==1) {
            auto &f=*self.fixture;
            CHECK(!f.broker.ProposeHandUse(f.owner,Proposal(f,f.owner,self.executor,2,2),self.queued,Now()));
            f.broker.DrainHandUses(); // must not create a nested selection round
            CHECK(ProposalState(f,f.owner,self.queued).stage==HIGGS_USE_PROPOSAL_PENDING);
        }
    }
};
void ProposalReentrancy()
{
    auto f=std::make_unique<Fixture>(); ProposalReentry reentry{f.get()}; uint64_t first=0;
    CHECK(!f->broker.Executor(f->owner,ProposalReentry::Notify,&reentry,reentry.executor));
    CHECK(!f->broker.ProposeHandUse(f->owner,Proposal(*f,f->owner,reentry.executor,1),first,Now()));
    f->broker.DrainHandUses(); CHECK(reentry.calls==1 && reentry.queued);
    f->broker.DrainHandUses(); CHECK(reentry.calls==2);
    CHECK(ProposalState(*f,f->owner,first).roundId<ProposalState(*f,f->owner,reentry.queued).roundId);
    f->broker.DrainHandUses(); CHECK(reentry.calls==2);
}
struct ProposalSlowCallback {
    unsigned calls=0;
    static void HIGGS_HAND_CALL Notify(const HIGGS_ActionRequest *request,void *context) {
        auto &self=*static_cast<ProposalSlowCallback *>(context);
        ++self.calls;
        if (request->physicalHands==HIGGS_HAND_LEFT)
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
};
void ProposalNotificationExpiry()
{
    auto f=std::make_unique<Fixture>(); ProposalSlowCallback slow; uint64_t executor=0,left=0,right=0;
    CHECK(!f->broker.Executor(f->owner,ProposalSlowCallback::Notify,&slow,executor));
    auto first=Proposal(*f,f->owner,executor,HIGGS_HAND_LEFT,1);
    auto second=Proposal(*f,f->owner,executor,HIGGS_HAND_RIGHT,2); second.use.durationMs=1;
    CHECK(!f->broker.ProposeHandUse(f->owner,first,left,Now()));
    CHECK(!f->broker.ProposeHandUse(f->owner,second,right,Now()));
    f->broker.DrainHandUses();
    CHECK(slow.calls==1);
    CHECK(ProposalState(*f,f->owner,left).use.active);
    const auto expired=ProposalState(*f,f->owner,right);
    CHECK(expired.stage==HIGGS_USE_PROPOSAL_ACCEPTED && !expired.use.active);
    f->broker.DrainHandUses(); CHECK(slow.calls==1);
}
