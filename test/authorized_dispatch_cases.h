// Included by the production authority fixture; uses its existing Fixture owner.
HIGGS_AuthorizedDispatchRequest DispatchRequest(const HIGGS_ActionRequest& request)
{
    return {sizeof(HIGGS_AuthorizedDispatchRequest),0,request.requestId,request.executor,request.epoch,request.grant};
}
struct DispatchContext
{
    Fixture* fixture;
    HIGGS_AuthorizedDispatchRequest* invocation;
    unsigned calls=0;
    uint64_t child=0;
    HIGGS_HandStatus outcome=HIGGS_HAND_OK;
    bool throws=false, lifecycle=false, slow=false, childPhysical=false;
};
HIGGS_HandStatus HIGGS_HAND_CALL DeferredDispatch(const HIGGS_ActionRequest* request,
                                                 const HIGGS_HandSnapshot* snapshot,void* context)
{
    auto& d=*static_cast<DispatchContext*>(context);auto& f=*d.fixture;++d.calls;
    CHECK(snapshot->epoch==request->epoch && snapshot->hands[0].grant && snapshot->hands[0].grant!=request->grant);
    HIGGS_HandSnapshot copied{};CHECK(!f.broker.Copy(f.owner,copied,Now()));
    CHECK(f.broker.InvokeAuthorized(f.owner,*d.invocation,DeferredDispatch,&d)==HIGGS_HAND_CONFLICT);
    HIGGS_ActionResult report{sizeof(report),HIGGS_ACTION_COMPLETED,request->requestId,
        request->executor,request->epoch,request->inputId,0,0,0,0};
    CHECK(f.broker.Report(f.owner,report)==HIGGS_HAND_PENDING);
    CHECK(f.broker.Release(f.owner,request->grant)==HIGGS_HAND_PENDING);
    CHECK(f.broker.Remove(f.owner,request->executor)==HIGGS_HAND_PENDING);
    CHECK(f.broker.Unregister(f.owner)==HIGGS_HAND_PENDING);
    if(d.childPhysical){
        HIGGS_PhysicalRequest child{sizeof(child),HIGGS_PHYSICAL_GRAB,HIGGS_HAND_LEFT,123,
            request->epoch,snapshot->hands[0].generation,request->grant,1000,0};
        CHECK(!f.broker.Physical(f.owner,child,d.child,Now()));
    }
    if(d.lifecycle){
        f.broker.ObserveWorld(0,true,Now());
        CHECK(f.broker.CopyPublished().epoch==request->epoch);
        CHECK(!f.broker.MutationAdmissionReady());
    }
    if(d.slow)std::this_thread::sleep_for(std::chrono::milliseconds(35));
    if(d.throws)throw 1;
    return d.outcome;
}
void AuthorizedDispatchOnce()
{
    for(unsigned mode=0;mode<4;++mode){
        auto f=std::make_unique<Fixture>();auto use=f->Use();auto r=DispatchRequest(use.authority);
        DispatchContext d{f.get(),&r};d.childPhysical=mode==0;
        d.outcome=mode==1?HIGGS_HAND_PENDING:mode==2?HIGGS_HAND_CONFLICT:HIGGS_HAND_OK;d.throws=mode==3;
        const auto expected=d.throws?HIGGS_HAND_INTERNAL_ERROR:d.outcome;
        CHECK(f->broker.InvokeAuthorized(f->owner,r,DeferredDispatch,&d)==expected && d.calls==1);
        HIGGS_ActionResult result{};CHECK(!f->broker.Result(f->owner,r.requestId,result));
        CHECK(result.stage==HIGGS_ACTION_ACCEPTED && !result.receiptFlags);
        CHECK(f->broker.InvokeAuthorized(f->owner,r,DeferredDispatch,&d)==HIGGS_HAND_CONFLICT && d.calls==1);
        if(d.child){
            HIGGS_ActionResult child{};CHECK(!f->broker.Result(f->owner,d.child,child));
            CHECK(child.stage==HIGGS_ACTION_ACCEPTED && !child.receiptFlags);
        }
        if(mode==0){
            // Only a factual provider report, after return, records completion.
            result.stage=HIGGS_ACTION_COMPLETED;
            CHECK(!f->broker.Report(f->owner,result));
            CHECK(!f->broker.Result(f->owner,r.requestId,result));
            CHECK(result.stage==HIGGS_ACTION_COMPLETED && !result.receiptFlags);
        }
        CHECK(!f->broker.Release(f->owner,r.grant));
    }
    std::puts("PASS authorized dispatch claims once, queues child physical work, never fabricates callback receipts");
}
void AuthorizedDispatchAdmission()
{
    auto f=std::make_unique<Fixture>();auto use=f->Use();auto r=DispatchRequest(use.authority);
    DispatchContext d{f.get(),&r};
    auto invalid=r;invalid.reserved=1;
    CHECK(f->broker.InvokeAuthorized(f->owner,invalid,DeferredDispatch,&d)==HIGGS_HAND_INVALID_ARGUMENT);
    CHECK(f->broker.InvokeAuthorized(f->owner,r,nullptr,&d)==HIGGS_HAND_INVALID_ARGUMENT);
    CHECK(f->broker.InvokeAuthorized(f->other,r,DeferredDispatch,&d)==HIGGS_HAND_WRONG_OWNER);
    invalid=r;++invalid.executor;
    CHECK(f->broker.InvokeAuthorized(f->owner,invalid,DeferredDispatch,&d)==HIGGS_HAND_WRONG_OWNER);
    invalid=r;++invalid.epoch;
    CHECK(f->broker.InvokeAuthorized(f->owner,invalid,DeferredDispatch,&d)==HIGGS_HAND_STALE);
    invalid=r;++invalid.grant;
    CHECK(f->broker.InvokeAuthorized(f->owner,invalid,DeferredDispatch,&d)==HIGGS_HAND_STALE);
    std::thread wrong([&]{CHECK(f->broker.InvokeAuthorized(f->owner,r,DeferredDispatch,&d)==HIGGS_HAND_WRONG_PHASE);});
    wrong.join();CHECK(!d.calls);
    // Every required hand must remain currently known and available.
    for(unsigned mode=0;mode<3;++mode){
        auto changed=f->hands;
        if(mode==0)changed[1].observationMask&=~HIGGS_OBS_PALM_VALID;
        if(mode==1)changed[1].unavailableReasons=HIGGS_HAND_REASON_RESTRICTION;
        if(mode==2)changed[1].observationMask|=HIGGS_OBS_SECONDARY_SUPPORT;
        f->broker.Publish(changed,Now());
        CHECK(f->broker.InvokeAuthorized(f->owner,r,DeferredDispatch,&d)==HIGGS_HAND_NOT_READY && !d.calls);
    }
    f->broker.Publish(f->hands,Now());
    f->broker.SetInputContext(1,Now());
    CHECK(f->broker.InvokeAuthorized(f->owner,r,DeferredDispatch,&d)==HIGGS_HAND_NOT_READY && !d.calls);
    f->broker.SetInputContext(0,Now());
    CHECK(!f->broker.InvokeAuthorized(f->owner,r,DeferredDispatch,&d) && d.calls==1);
    CHECK(!f->broker.Release(f->owner,r.grant));
    std::puts("PASS authorized dispatch owner/thread/scope/grant and bilateral resource admission");
}
void HIGGS_HAND_CALL ProposeDeferredDispatch(const HIGGS_HandInput* input,void* context)
{
    auto& f=*static_cast<Fixture*>(context);
    HIGGS_ActionCandidate candidate{sizeof(candidate),input->physicalHand,input->inputId,input->epoch,
        input->handGeneration,f.executor,1,2,3,1000,0};
    uint64_t request=0;CHECK(!f.broker.Submit(f.owner,candidate,request,Now()));
}
void AuthorizedDispatchInputBoundary()
{
    auto f=std::make_unique<Fixture>();uint64_t recognizer=0;
    CHECK(!f->broker.Recognizer(f->owner,ProposeDeferredDispatch,f.get(),recognizer));
    CHECK(f->broker.Input(HIGGS_HAND_LEFT,HIGGS_INPUT_GRIP,true,Now()));
    auto r=DispatchRequest(f->notification);DispatchContext d{f.get(),&r};
    // Open+close between selection and safe-phase invocation retires the input.
    f->broker.SetInputContext(1,Now());f->broker.SetInputContext(0,Now());
    CHECK(f->broker.InvokeAuthorized(f->owner,r,DeferredDispatch,&d)!=HIGGS_HAND_OK && !d.calls);
    std::puts("PASS selected deferred input cannot execute across a menu-context transition");
}
void AuthorizedDispatchLifecycle()
{
    for(bool lifecycle:{false,true}){
        auto f=std::make_unique<Fixture>();auto use=f->Use(lifecycle?1000:25);
        auto r=DispatchRequest(use.authority);DispatchContext d{f.get(),&r};
        d.lifecycle=lifecycle;d.slow=!lifecycle;
        CHECK(!f->broker.InvokeAuthorized(f->owner,r,DeferredDispatch,&d) && d.calls==1);
        HIGGS_ActionResult result{};CHECK(!f->broker.Result(f->owner,r.requestId,result));
        CHECK(result.stage==HIGGS_ACTION_CANCELLED && !result.receiptFlags);
        if(lifecycle)CHECK(f->broker.CopyPublished().epoch>r.epoch && !f->broker.CopyPublished().worldReady);
        CHECK(f->broker.InvokeAuthorized(f->owner,r,DeferredDispatch,&d)==HIGGS_HAND_CONFLICT && d.calls==1);
    }
    std::puts("PASS deferred dispatch holds authority through callback then drains ordinary expiry/world reset");
}
