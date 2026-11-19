#include "hand_abi_boundary.h"
#include "own_contact.h"
#include "hand_broker.h"
#include <atomic>
#include <cstring>
namespace HiggsHands {
namespace {std::atomic<OwnContactNativeObserver> observer{nullptr};}
void InstallOwnContactNativeObserver(OwnContactNativeObserver value){observer.store(value,std::memory_order_release);}
HIGGS_HandStatus ObserveOwnContactNative(const HIGGS_NativeOwnContact& input,uint64_t& out){const auto call=observer.load(std::memory_order_acquire);return call?call(input,out):HIGGS_HAND_NOT_READY;}
}
namespace {

template<class T> bool Output(T* value){return value&&value->structSize>=sizeof(T);}
HIGGS_HandStatus HIGGS_HAND_CALL Request(uint64_t owner,const HIGGS_CosmeticContactPolicy* p,HIGGS_CosmeticContactState* out){if(!p||!Output(out))return HIGGS_HAND_INVALID_ARGUMENT;return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return HiggsHands::OwnContacts().Request(owner,*p,value,HiggsHands::Now());
    });}
HIGGS_HandStatus HIGGS_HAND_CALL End(uint64_t owner,uint64_t id,uint64_t epoch){return HiggsAbi::Guard([&]{return HiggsHands::OwnContacts().End(owner,id,epoch,HiggsHands::Now());});}
HIGGS_HandStatus HIGGS_HAND_CALL Copy(uint64_t owner,uint64_t id,HIGGS_CosmeticContactState* out){if(!Output(out))return HIGGS_HAND_INVALID_ARGUMENT;return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return HiggsHands::OwnContacts().Copy(owner,id,value,HiggsHands::Now());
    });}
HIGGS_HandStatus HIGGS_HAND_CALL Contacts(uint64_t owner,const HIGGS_CosmeticContactQuery* q,HIGGS_CosmeticContact* out,uint32_t capacity,uint32_t* required){if(!q||!required)return HIGGS_HAND_INVALID_ARGUMENT;return HiggsAbi::Guard([&]{uint32_t value=0;auto s=HiggsHands::OwnContacts().Contacts(owner,*q,out,capacity,value,HiggsHands::Now());if(s==HIGGS_HAND_OK||s==HIGGS_HAND_BUFFER_TOO_SMALL)*required=value;return s;});}
HIGGS_HandStatus HIGGS_HAND_CALL Observe(const HIGGS_NativeOwnContact* input,uint64_t* out){if(!input||!out)return HIGGS_HAND_INVALID_ARGUMENT;return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return HiggsHands::ObserveOwnContactNative(*input,value);
    });}
HIGGS_HandStatus HIGGS_HAND_CALL Items(uint64_t owner,HIGGS_ContactItemFrame* out){if(!Output(out))return HIGGS_HAND_INVALID_ARGUMENT;return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return HiggsHands::OwnContacts().Items(owner,value);
    });}
}
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryCosmeticContactInterface(uint32_t version,uint32_t bytes,HIGGS_CosmeticContactInterface* out){if(version!=1)return HIGGS_HAND_UNSUPPORTED_VERSION;if(!out)return HIGGS_HAND_INVALID_ARGUMENT;if(bytes<sizeof(*out))return HIGGS_HAND_BUFFER_TOO_SMALL;return HiggsAbi::Guard([&]{const HIGGS_CosmeticContactInterface table{sizeof(table),1,HiggsHands::OwnContacts().Capabilities(),Request,End,Copy,Contacts,Items};std::memcpy(out,&table,sizeof(table));return HIGGS_HAND_OK;});}
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryOwnContactNativeInterface(uint32_t version,uint32_t bytes,HIGGS_OwnContactNativeInterface* out){if(version!=1)return HIGGS_HAND_UNSUPPORTED_VERSION;if(!out)return HIGGS_HAND_INVALID_ARGUMENT;if(bytes<sizeof(*out))return HIGGS_HAND_BUFFER_TOO_SMALL;return HiggsAbi::Guard([&]{const HIGGS_OwnContactNativeInterface table{sizeof(table),1,HiggsHands::OwnContacts().Capabilities(),Observe};std::memcpy(out,&table,sizeof(table));return HIGGS_HAND_OK;});}
