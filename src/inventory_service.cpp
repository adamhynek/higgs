#include "synergy_inventory_native.h"
#include "hand_broker.h"
#include "hand_transactions.h"
#include <cstring>
namespace HiggsHands {
InventoryService& Inventory(){static InventoryService service;return service;}
void InventoryService::Install(InventoryEngine* engine){std::lock_guard lock(mutex_);engine_=engine;}
uint64_t InventoryService::Capabilities(){std::lock_guard lock(mutex_);return engine_?engine_->Capabilities():0;}
void InventoryService::Sync(){
    const auto epoch=Service().Epoch();
    if(epoch!=epoch_){tickets_={};active_={};if(engine_)engine_->Reset();epoch_=epoch;}
}
HIGGS_HandStatus InventoryService::Request(uint64_t owner,const HIGGS_InventorySelectionRequest& r,uint64_t& out){
    auto phase=Service().NativePhase();std::lock_guard lock(mutex_);
    if(!Service().ClientValid(owner))return HIGGS_HAND_INVALID_CLIENT;
    if(r.structSize<sizeof(r)||!r.baseFormId||!r.quantity||r.quantity>INT32_MAX||r.reserved[0]||r.reserved[1])return HIGGS_HAND_INVALID_ARGUMENT;
    if(r.epoch!=Service().Epoch())return HIGGS_HAND_STALE;
    if(epoch_!=r.epoch)return HIGGS_HAND_NOT_READY;
    if(!engine_)return HIGGS_HAND_NOT_READY;
    for(auto& t:tickets_)if(!t.owner){t.owner=owner;t.request=r;t.value.structSize=sizeof(t.value);t.value.state=HIGGS_SELECTION_QUEUED;t.value.ticket=next_++;t.value.epoch=epoch_;out=t.value.ticket;return HIGGS_HAND_OK;}
    return HIGGS_HAND_CAPACITY;
}
HIGGS_HandStatus InventoryService::Copy(uint64_t owner,uint64_t id,HIGGS_InventorySelection& out){
    auto phase=Service().NativePhase();std::lock_guard lock(mutex_);
    if(!Service().ClientValid(owner))return HIGGS_HAND_INVALID_CLIENT;
    if(epoch_!=Service().Epoch())return HIGGS_HAND_STALE;
    for(auto& t:tickets_)if(t.owner&&t.value.ticket==id){if(t.owner!=owner)return HIGGS_HAND_WRONG_OWNER;out=t.value;return HIGGS_HAND_OK;}return HIGGS_HAND_NOT_FOUND;
}
HIGGS_HandStatus InventoryService::Release(uint64_t owner,uint64_t id){
    auto phase=Service().NativePhase();std::lock_guard lock(mutex_);
    if(!Service().ClientValid(owner))return HIGGS_HAND_INVALID_CLIENT;
    if(epoch_!=Service().Epoch())return HIGGS_HAND_STALE;
    for(auto& t:tickets_)if(t.owner&&t.value.ticket==id){if(t.owner!=owner)return HIGGS_HAND_WRONG_OWNER;if(t.value.item.instance&&Transactions().ItemInUse(t.value.item.instance))return HIGGS_HAND_PENDING;Transactions().InvalidateInventory(owner,id);t.value.state=HIGGS_SELECTION_INVALID;t.value.reason=HIGGS_HAND_STALE;t.retired=true;return HIGGS_HAND_OK;}return HIGGS_HAND_NOT_FOUND;
}
HIGGS_HandStatus InventoryService::Executor(uint64_t owner,uint64_t& out){
    auto phase=Service().NativePhase();std::lock_guard lock(mutex_);
    if(!Service().ClientValid(owner))return HIGGS_HAND_INVALID_CLIENT;
    if(!engine_)return HIGGS_HAND_NOT_READY;
    for(auto& o:owners_)if(o.client==owner){if(!Service().ExecutorValid(owner,o.executor,4))return HIGGS_HAND_STALE;out=o.executor;return HIGGS_HAND_OK;}
    for(auto& o:owners_)if(!o.client){o.service=this;o.client=owner;auto s=Service().TransferExecutor(owner,Execute,&o,o.executor);if(s){o={};return s;}out=o.executor;return HIGGS_HAND_OK;}return HIGGS_HAND_CAPACITY;
}
void InventoryService::Drain(){
    // Caller is the existing native main maintenance phase, not an arbitrary API thread.
    auto phase=Service().NativePhase();std::lock_guard lock(mutex_);Sync();
    if(!engine_)return;
    for(auto& a:active_)if(a.request){
        HIGGS_TransferState state{};
        // Public Copy rejects closed clients. That is not permission to destroy
        // the retained native source of an issued or recovery-required mutation.
        if(!Transactions().NativePinRequired(a.owner,a.request,epoch_,state)){
            engine_->Forget(a.request);a={};continue;
        }
        if(a.pending&&state.stage!=HIGGS_TX_RECOVERY_REQUIRED&&Service().ClientValid(a.owner)){
            auto result=engine_->Poll(a.execution);
            if(result.status!=HIGGS_HAND_PENDING){Report(a.execution,a.permit,result);a.pending=false;}
        }
    }
    for(auto& t:tickets_)if(t.owner){
        if(!Service().ClientValid(t.owner)){Transactions().InvalidateInventory(t.owner,t.value.ticket);if(Transactions().RetireInventory(t.owner,t.value.ticket))t={};continue;}
        if(t.value.state==HIGGS_SELECTION_READY&&!engine_->Current(t.native)){
            // A transaction may already hold this native identity. Its callback performs
            // another fresh resolve; do not mutate the locked original declaration.
            Transactions().InvalidateInventory(t.owner,t.value.ticket);t.value.state=HIGGS_SELECTION_INVALID;t.value.reason=HIGGS_HAND_STALE;
        }
        if(t.retired&&Transactions().RetireInventory(t.owner,t.value.ticket)){t={};continue;}
        if(t.value.state!=HIGGS_SELECTION_QUEUED)continue;
        uint32_t container=0;auto s=engine_->Select(t.request,t.native,container);
        if(!s)s=Transactions().ObserveInventory(t.owner,t.value.ticket,1,t.request.baseFormId,t.request.quantity,t.value.item);
        t.value.containerRefId=container;t.value.reason=s;t.value.state=s?HIGGS_SELECTION_INVALID:HIGGS_SELECTION_READY;
    }
    for(auto& o:owners_)if(o.client&&!Service().ClientValid(o.client)){Service().Remove(o.client,o.executor);o={};}
}
void HIGGS_HAND_CALL InventoryService::Execute(const HIGGS_TransferExecution* x,void* context){if(x&&context){auto& o=*static_cast<Owner*>(context);o.service->ExecuteFor(o,*x);}}
void InventoryService::ExecuteFor(Owner& owner,const HIGGS_TransferExecution& x){
    std::shared_ptr<InventorySelectionHandle> selection;InventoryEngine* engine=nullptr;
    {
        std::lock_guard lock(mutex_);Sync();engine=engine_;
        if(x.epoch!=epoch_||x.current.owner!=owner.client||x.executor!=owner.executor||!engine)return;
        if(x.request.original.kind==HIGGS_ITEM_NATIVE_INVENTORY){
            for(auto& t:tickets_)if(t.owner==owner.client&&t.value.ticket==x.request.original.providerKey&&t.value.item.instance==x.request.original.instance&&t.value.item.generation==x.request.original.generation){selection=t.native;break;}
            if(!selection){Transactions().Cancel(owner.client,x.requestId,x.epoch);return;}
        }
        bool recorded=false;for(auto& a:active_)if(a.request==x.requestId){recorded=true;break;}
        if(!recorded){for(auto& a:active_)if(!a.request){a.owner=owner.client;a.request=x.requestId;recorded=true;break;}}
        if(!recorded){Transactions().Cancel(owner.client,x.requestId,x.epoch);return;}
    }
    struct Permit final:InventoryPermit {
        const HIGGS_TransferExecution& x;bool started=false;uint64_t value=0;
        explicit Permit(const HIGGS_TransferExecution& input):x(input){}
        // Broker::InvokeTransfer already fences the complete callback, including
        // synchronous engine events and deferred world changes. Do not nest a
        // second admission that would reject its existing nativeStarted marker.
        bool Begin() override {if(started||Transactions().Begin(x.current.owner,x.requestId,x.epoch,value))return false;started=true;return true;}
    } permit(x);
    InventoryOutcome result{};
    try{result=engine->Execute(x,selection,permit);}catch(...){result.status=HIGGS_HAND_INTERNAL_ERROR;}
    if(!permit.value){Transactions().Cancel(owner.client,x.requestId,x.epoch);return;}
    if(result.status==HIGGS_HAND_PENDING){std::lock_guard lock(mutex_);for(auto& a:active_)if(a.request==x.requestId){a.execution=x;a.permit=permit.value;a.pending=true;return;}}
    Report(x,permit.value,result);
}
void InventoryService::Report(const HIGGS_TransferExecution& x,uint64_t permit,const InventoryOutcome& result){
    HIGGS_TransferReceipt receipt{};receipt.structSize=sizeof(receipt);receipt.kind=result.status?HIGGS_TX_EXECUTION_FAILED:result.receipt;receipt.requestId=x.requestId;receipt.executor=x.executor;receipt.epoch=x.epoch;receipt.mutationPermit=permit;receipt.original=x.request.original;receipt.quantity=x.request.quantity;receipt.actualRefId=result.refId;receipt.instancePreserved=result.preserved?1:0;receipt.reason=result.status;receipt.actualEquippedFormId=result.equippedFormId;
    Transactions().Report(x.current.owner,receipt);
}
}
namespace {
template<class F> HIGGS_HandStatus Safe(F f)noexcept{try{return f();}catch(...){return HIGGS_HAND_INTERNAL_ERROR;}}
HIGGS_HandStatus HIGGS_HAND_CALL Select(uint64_t c,const HIGGS_InventorySelectionRequest* r,uint64_t* out){if(!r||!out)return HIGGS_HAND_INVALID_ARGUMENT;return Safe([&]{return HiggsHands::Inventory().Request(c,*r,*out);});}
HIGGS_HandStatus HIGGS_HAND_CALL Copy(uint64_t c,uint64_t t,HIGGS_InventorySelection* out){if(!out)return HIGGS_HAND_INVALID_ARGUMENT;if(out->structSize<sizeof(*out)){out->structSize=sizeof(*out);return HIGGS_HAND_BUFFER_TOO_SMALL;}return Safe([&]{HIGGS_InventorySelection value{};auto s=HiggsHands::Inventory().Copy(c,t,value);if(!s)std::memcpy(out,&value,sizeof(value));return s;});}
HIGGS_HandStatus HIGGS_HAND_CALL Release(uint64_t c,uint64_t t){return Safe([&]{return HiggsHands::Inventory().Release(c,t);});}
HIGGS_HandStatus HIGGS_HAND_CALL Executor(uint64_t c,uint64_t* out){if(!out)return HIGGS_HAND_INVALID_ARGUMENT;return Safe([&]{return HiggsHands::Inventory().Executor(c,*out);});}
}
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryInventoryInterface(uint32_t version,uint32_t bytes,HIGGS_InventoryInterface* out){
    if(version!=HIGGS_INVENTORY_VERSION)return HIGGS_HAND_UNSUPPORTED_VERSION;if(!out)return HIGGS_HAND_INVALID_ARGUMENT;if(bytes<sizeof(*out))return HIGGS_HAND_BUFFER_TOO_SMALL;
    return Safe([&]{HIGGS_InventoryInterface value{sizeof(value),version,HiggsHands::Inventory().Capabilities(),Select,Copy,Release,Executor};std::memcpy(out,&value,sizeof(value));return HIGGS_HAND_OK;});
}
