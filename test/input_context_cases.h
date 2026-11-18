// Real Broker/BeginControllerFrame/FinishControllerFrame, no menu model.
struct ContextObserver { unsigned calls=0; uint64_t token=0, owner=0; bool remove=false,reenter=false; HIGGS_InputContextState last{}; };
void HIGGS_HAND_CALL ContextChanged(const HIGGS_InputContextState* old,const HIGGS_InputContextState* state,void* data) {
    auto& c=*static_cast<ContextObserver*>(data);++c.calls;CHECK(!c.last.generation||state->generation>c.last.generation);c.last=*state;
    CHECK(state->generation>old->generation);
    if(c.remove)CHECK(Service().Remove(c.owner,c.token)==HIGGS_HAND_PENDING);
    if(c.reenter){c.reenter=false;Service().SetInputContext(2,state->changedAtNs+1);}
}
void HIGGS_HAND_CALL ChangeDuringRecognition(const HIGGS_InputEvent* event,void*) {
    if((event->flags&HIGGS_INPUT_ACTIONABLE)&&event->kind==HIGGS_RAW_PRESS) {
        Service().SetInputContext(1,event->frame.observedAtNs);
        Service().SetInputContext(0,event->frame.observedAtNs+1);
    }
}
HIGGS_HandStatus HIGGS_HAND_CALL CountMutation(const HIGGS_ActionRequest*,const HIGGS_HandSnapshot*,void* data) {
    ++*static_cast<unsigned*>(data);return HIGGS_HAND_OK;
}
void Contexts() {
    Service().SetInputContext(0,Now());Service().ObserveWorld(200,false,Now());PublishHands();Service().MarkMainThread();
    uint64_t at=Now()+UINT64_C(10000000000),out=0;
    Consumer c;Setup(c,"context-hold",7,HIGGS_GESTURE_HOLD);
    HIGGS_InputContextInterface api{};CHECK(!HIGGS_QueryInputContextInterface(1,sizeof(api),&api)&&api.capabilities==7);
    auto original=api;CHECK(HIGGS_QueryInputContextInterface(2,sizeof(api),&api)==HIGGS_HAND_UNSUPPORTED_VERSION&&!std::memcmp(&api,&original,sizeof(api)));
    CHECK(HIGGS_QueryInputContextInterface(1,sizeof(api)-1,&api)==HIGGS_HAND_BUFFER_TOO_SMALL);
    HIGGS_InputContextState state{};state.structSize=sizeof(state);CHECK(api.CopyContext(UINT64_MAX,&state)==HIGGS_HAND_INVALID_CLIENT);
    ContextObserver observer;observer.owner=c.client;CHECK(!api.Subscribe(c.client,ContextChanged,&observer,&observer.token));
    // Initially/reconnected held controls are unknown sequences, never new Down.
    Frame(Bit(7)|Bit(2)|Bit(33),at++,out);CHECK(!out&&!c.executions&&!c.lastRequest);
    Frame(Bit(7)|Bit(2)|Bit(33),at++,out,2);CHECK(!out&&!c.executions);
    Frame(0,at++,out);Frame(0,at++,out,2);
    Frame(Bit(7),at,out);at+=10000000;CHECK(!out&&c.lastRequest&&!c.executions);
    const auto waiting=c.lastRequest;Service().SetInputContext(1,at++);CHECK(observer.calls==1&&observer.last.kind==HIGGS_CONTEXT_UI);
    HIGGS_InputRecognition recognition{};CHECK(!Service().CopyInputRecognition(c.client,waiting,recognition)&&recognition.stage==HIGGS_RECOGNITION_CANCELLED);
    Frame(Bit(7),at++,out);CHECK(!out&&!c.executions);Frame(0,at++,out);CHECK(!out);
    const auto actionable=c.actionable;Frame(Bit(7)|Bit(2)|Bit(33),at++,out);CHECK(out==(Bit(7)|Bit(2)|Bit(33))&&c.actionable==actionable);
    Service().SetInputContext(0,at++);Frame(Bit(7)|Bit(2)|Bit(33),at++,out);CHECK(!out&&c.actionable==actionable);
    Frame(0,at++,out);CHECK(!out&&c.actionable==actionable);
    c.gesture=0;Frame(Bit(7),at++,out);CHECK(c.executions==1&&!out);Frame(0,at++,out);
    // A session keeps its exact grant through UI; fresh UI controls bypass it.
    c.complete=false;HIGGS_HandUseRequest use{};use.structSize=sizeof(use);use.kind=HIGGS_USE_HAND_GOALS;use.physicalHands=3;use.durationMs=60000;use.epoch=Service().Epoch();use.executor=c.executor;
    HIGGS_HandUseState session{};CHECK(!Service().HandUse(c.client,use,session,at));c.parent=session.authority.grant;
    Service().SetInputContext(1,at++);Frame(Bit(7)|Bit(33),at++,out);CHECK(out==(Bit(7)|Bit(33)));
    HIGGS_HandUseState renewed{};CHECK(!Service().ReadHandUse(c.client,session.authority.requestId,use.epoch,60000,renewed,at)&&renewed.active&&renewed.authority.grant==c.parent);
    HIGGS_HandUseState refused{};CHECK(Service().HandUse(c.client,use,refused,at)==HIGGS_HAND_CONFLICT);
    CHECK(!Service().MutationAdmissionReady()&&Service().MutationAdmissionReady(true));
    CHECK(Service().CopyPublished().hands[0].grant==c.parent&&Service().CopyPublished().hands[1].grant==c.parent);
    const auto holdSnapshot=Service().CopyPublished();std::array<HIGGS_HandRecord,2> heldRecords{holdSnapshot.hands[0],holdSnapshot.hands[1]};
    heldRecords[0].heldRefId=501;heldRecords[0].unavailableReasons=HIGGS_HAND_REASON_MENU;Service().Publish(heldRecords,at++);
    const auto occupied=Service().CopyPublished();HIGGS_PhysicalRequest release{};release.structSize=sizeof(release);release.kind=HIGGS_PHYSICAL_RELEASE;
    release.physicalHand=1;release.itemRefId=501;release.epoch=occupied.epoch;release.handGeneration=occupied.hands[0].generation;release.grant=c.parent;release.timeoutMs=1000;
    uint64_t released=0;CHECK(!Service().Physical(c.client,release,released,at));unsigned releaseCalls=0;
    Service().DrainPhysical([](const HIGGS_PhysicalRequest& request,void* data){CHECK(request.kind==HIGGS_PHYSICAL_RELEASE);++*static_cast<unsigned*>(data);return true;},&releaseCalls,at);
    CHECK(releaseCalls==1);HIGGS_ActionResult releaseResult{};CHECK(!Service().Result(c.client,released,releaseResult)&&releaseResult.stage==HIGGS_ACTION_COMMITTED);
    CHECK(!Service().EndHandUse(c.client,session.authority.requestId,use.epoch));c.parent=0;PublishHands();
    // Context frame identifies the immutable raw frame, independently of pose ID.
    HIGGS_ControllerFrame raw{};CHECK(InputFrames().Latest(1,raw));HIGGS_InputContextFrame stamp{};stamp.structSize=sizeof(stamp);
    CHECK(!api.CopyFrameContext(c.client,raw.frameId,&stamp)&&stamp.kind==HIGGS_CONTEXT_UI&&stamp.generation==observer.last.generation);
    const auto uiFrame=stamp;Service().SetInputContext(0,at++);CHECK(!api.CopyFrameContext(c.client,raw.frameId,&stamp)&&!std::memcmp(&stamp,&uiFrame,sizeof(stamp)));
    Frame(0,at++,out);Frame(0,at++,out,2);
    // A previously dispatched recognizer may queue main-phase work. A changed
    // context rejects that uncommitted fence even after returning to gameplay.
    Frame(Bit(7),at++,out);CHECK(c.executions==2);const auto accepted=c.executed;
    Service().SetInputContext(1,at++);Service().SetInputContext(0,at++);
    HIGGS_GrantedMutationRequest mutation{};mutation.structSize=sizeof(mutation);mutation.epoch=accepted.epoch;mutation.executor=accepted.executor;mutation.requestId=accepted.requestId;mutation.grant=accepted.grant;mutation.commitStage=HIGGS_ACTION_COMPLETED;
    unsigned writes=0;HIGGS_ActionResult receipt{};CHECK(Service().WithAuthorizedAction(c.client,mutation,CountMutation,&writes,receipt)==HIGGS_HAND_STALE&&!writes);
    HIGGS_ActionRequest passive{};CHECK(Service().ReserveEffectAuthority(c.client,accepted.epoch,1000,passive,Now(),accepted.grant,1)!=HIGGS_HAND_OK&&!passive.requestId);
    c.parent=accepted.grant;Frame(0,at++,out);Frame(Bit(7),at++,out);CHECK(c.submitted==HIGGS_HAND_CONFLICT&&c.executions==2);c.parent=0;
    CHECK(!Service().Release(c.client,accepted.grant));Frame(0,at++,out);
    // Reentrant open+close during the recognition round cannot dispatch old Down.
    HIGGS_InputRecognizerSpec changes{};changes.structSize=sizeof(changes);changes.physicalHands=1;changes.controlMask=Bit(7);changes.callback=ChangeDuringRecognition;uint64_t changer=0;
    CHECK(!Service().InputRecognizer(c.client,changes,changer));Consumer after;Setup(after,"after-context-change",7);
    const auto calls=c.executions;Frame(Bit(7),at++,out);CHECK(c.executions==calls&&!after.actionable&&!out);CHECK(!Service().Remove(c.client,changer));CHECK(!Service().Unregister(after.client));Frame(0,at++,out);
    ContextObserver second;second.owner=c.client;CHECK(!api.Subscribe(c.client,ContextChanged,&second,&second.token));observer.reenter=true;
    Service().SetInputContext(1,at++);CHECK(observer.last.generation==second.last.generation&&observer.last.activeMenus==2);CHECK(!Service().Remove(c.client,second.token));Service().SetInputContext(0,at++);
    observer.remove=true;Service().SetInputContext(1,at++);const auto removedCalls=observer.calls;Service().SetInputContext(2,at++);CHECK(observer.calls==removedCalls);CHECK(!Service().Remove(c.client,observer.token));
    Service().SetInputContext(0,at++);Service().ObserveWorld(201,false,Now());CHECK(api.CopyFrameContext(c.client,raw.frameId,&stamp)==HIGGS_HAND_STALE);
    CHECK(!Service().Unregister(c.client));state.structSize=sizeof(state);CHECK(api.CopyContext(c.client,&state)==HIGGS_HAND_INVALID_CLIENT);
    // A queued explicit release is cleanup, not a new gameplay start.
    Broker cleanup;cleanup.ObserveWorld(301,false,Now());HIGGS_HandClientInfo info{};info.structSize=sizeof(info);std::memcpy(info.name,"queued-release",15);uint64_t owner=0;
    CHECK(!cleanup.Register(info,owner));std::array<HIGGS_HandRecord,2> records{};for(unsigned i=0;i<2;++i){records[i].physicalHand=i+1;records[i].observationMask=HIGGS_OBS_HELD_KNOWN|HIGGS_OBS_EQUIPMENT_KNOWN|HIGGS_OBS_PALM_VALID|HIGGS_OBS_TRACKING_VALID;}
    records[0].heldRefId=600;cleanup.Publish(records,Now());const auto beforeCleanup=cleanup.CopyPublished();release.epoch=beforeCleanup.epoch;release.handGeneration=beforeCleanup.hands[0].generation;release.itemRefId=600;release.grant=0;
    CHECK(!cleanup.Physical(owner,release,released,Now()));cleanup.SetInputContext(1,Now());releaseCalls=0;
    cleanup.DrainPhysical([](const HIGGS_PhysicalRequest&,void* data){++*static_cast<unsigned*>(data);return true;},&releaseCalls,Now());CHECK(releaseCalls==1);
}
