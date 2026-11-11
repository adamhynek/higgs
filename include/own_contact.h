#pragma once
#include "higgsowncontactnative.h"
#include <array>
#include <atomic>
#include <mutex>
namespace HiggsHands {
struct OwnContactFrame {
    uint64_t epoch=0,observation=0,observedAt=0,invalidationTicket=0;
    std::array<HIGGS_ContactItem,4> items{};
    std::array<uint32_t,4> sources{};
    uint32_t count=0;
};
class OwnContactService {
public:
    HIGGS_HandStatus Request(uint64_t,const HIGGS_CosmeticContactPolicy&,HIGGS_CosmeticContactState&,uint64_t);
    HIGGS_HandStatus End(uint64_t,uint64_t,uint64_t,uint64_t);
    HIGGS_HandStatus Copy(uint64_t,uint64_t,HIGGS_CosmeticContactState&,uint64_t);
    HIGGS_HandStatus Contacts(uint64_t,const HIGGS_CosmeticContactQuery&,HIGGS_CosmeticContact*,uint32_t,uint32_t&,uint64_t);
    bool Publish(const OwnContactFrame&,uint64_t capabilities,uint64_t);
    void Invalidate();
    uint64_t SourceTicket()const{return invalidationTicket_.load(std::memory_order_acquire);}
    uint64_t Capabilities();
    HIGGS_HandStatus Items(uint64_t,HIGGS_ContactItemFrame&);
    // Physics path: only this service lock; no broker/engine call or callback.
    HIGGS_HandStatus Observe(const OwnContactFrame&,const HIGGS_ContactItem&,const HIGGS_ContactItem&,const HIGGS_NativeOwnContact&,uint64_t,uint64_t&);
private:
    struct Stamp {HIGGS_ContactItem left{},right{};uint64_t frame=0,tick=0,at=0;};
    struct Lease {HIGGS_CosmeticContactPolicy policy{};HIGGS_CosmeticContactState state{};std::array<Stamp,8> recent{};uint32_t next=0;};
    void Expire(uint64_t);
    Lease* Find(uint64_t);
    bool Current(const HIGGS_ContactItem&)const;
    std::mutex mutex_;
    std::atomic<uint64_t> invalidationTicket_{1};
    OwnContactFrame frame_{};
    bool ready_=false;
    uint64_t capabilities_=0,next_=1,sequence_=1;
    std::array<Lease,32> leases_{};
    std::array<HIGGS_CosmeticContact,512> contacts_{};
    size_t count_=0,write_=0;
};
OwnContactService& OwnContacts();
using OwnContactNativeObserver=HIGGS_HandStatus(*)(const HIGGS_NativeOwnContact&,uint64_t&);
void InstallOwnContactNativeObserver(OwnContactNativeObserver);
void InvalidateNativeOwnContacts();
void PublishNativeOwnContacts();
}
