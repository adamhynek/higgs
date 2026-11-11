#pragma once
#include "higgsinventory.h"
#include <array>
#include <memory>
#include <mutex>
namespace HiggsHands {
// Opaque engine-owned retained selection, never a caller-supplied native address.
struct InventorySelectionHandle { virtual ~InventorySelectionHandle()=default; };
struct InventoryOutcome {
    HIGGS_HandStatus status=HIGGS_HAND_UNSUPPORTED;
    uint32_t receipt=0, refId=0, equippedFormId=0;
    bool preserved=false;
};
struct InventoryPermit {
    virtual ~InventoryPermit()=default;
    virtual bool Begin()=0;
};
class InventoryEngine {
  public:
    virtual ~InventoryEngine()=default;
    virtual uint64_t Capabilities() const=0;
    virtual HIGGS_HandStatus Select(const HIGGS_InventorySelectionRequest&,std::shared_ptr<InventorySelectionHandle>&,uint32_t&)=0;
    virtual bool Current(const std::shared_ptr<InventorySelectionHandle>&)=0;
    virtual InventoryOutcome Execute(const HIGGS_TransferExecution&,const std::shared_ptr<InventorySelectionHandle>&,InventoryPermit&)=0;
    virtual InventoryOutcome Poll(const HIGGS_TransferExecution&)=0;
    virtual void Forget(uint64_t)=0;
    virtual void Reset()=0;
};
class InventoryService {
  public:
    HIGGS_HandStatus Request(uint64_t,const HIGGS_InventorySelectionRequest&,uint64_t&);
    HIGGS_HandStatus Copy(uint64_t,uint64_t,HIGGS_InventorySelection&);
    HIGGS_HandStatus Release(uint64_t,uint64_t);
    HIGGS_HandStatus Executor(uint64_t,uint64_t&);
    void Install(InventoryEngine*);
    uint64_t Capabilities();
    void Drain();
  private:
    struct Ticket {
        uint64_t owner=0;
        HIGGS_InventorySelectionRequest request{};
        HIGGS_InventorySelection value{};
        std::shared_ptr<InventorySelectionHandle> native;
        bool retired=false;
    };
    struct Owner { InventoryService* service=nullptr; uint64_t client=0,executor=0; };
    struct Active { uint64_t owner=0,request=0,permit=0; HIGGS_TransferExecution execution{}; bool pending=false; };
    static void HIGGS_HAND_CALL Execute(const HIGGS_TransferExecution*,void*);
    void ExecuteFor(Owner&,const HIGGS_TransferExecution&);
    void Report(const HIGGS_TransferExecution&,uint64_t,const InventoryOutcome&);
    void Sync();
    std::mutex mutex_;
    std::array<Ticket,128> tickets_{};
    std::array<Owner,64> owners_{};
    std::array<Active,128> active_{};
    InventoryEngine* engine_=nullptr;
    uint64_t epoch_=0,next_=1;
};
InventoryService& Inventory();
// Native main-phase pump; installed engine capabilities are absent in portable-only builds.
void DrainNativeInventory();
}
