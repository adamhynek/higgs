#include "hand_parts.h"
#include "hand_broker.h"
#include "higgshandpartsnative.h"
#include <atomic>
namespace {
std::atomic<HiggsHands::HandPartNativeResolver> nativeResolver{nullptr};
template<class F> HIGGS_HandStatus Guard(F f)noexcept{try{return f();}catch(...){return HIGGS_HAND_INTERNAL_ERROR;}}
template<class T> bool Sized(T* p){if(!p)return false;if(p->structSize<sizeof(T)){p->structSize=sizeof(T);return false;}return true;}
HIGGS_HandStatus HIGGS_HAND_CALL Sources(uint64_t c,uint32_t h,HIGGS_HandPartSources* p){if(!p)return HIGGS_HAND_INVALID_ARGUMENT;if(!Sized(p))return HIGGS_HAND_BUFFER_TOO_SMALL;return Guard([&]{HIGGS_HandPartSources out{};auto s=HiggsHands::HandParts().Sources(c,h,out,HiggsHands::Now());if(!s)*p=out;return s;});}
HIGGS_HandStatus HIGGS_HAND_CALL Set(uint64_t c,const HIGGS_HandPartSetRequest* r,HIGGS_HandPartSetState* p){if(!r||!p)return HIGGS_HAND_INVALID_ARGUMENT;if(!Sized(p))return HIGGS_HAND_BUFFER_TOO_SMALL;return Guard([&]{HIGGS_HandPartSetState out{};auto s=HiggsHands::HandParts().Set(c,*r,out,HiggsHands::Now());if(!s)*p=out;return s;});}
HIGGS_HandStatus HIGGS_HAND_CALL End(uint64_t c,uint64_t id,uint64_t e){return Guard([&]{return HiggsHands::HandParts().End(c,id,e);});}
HIGGS_HandStatus HIGGS_HAND_CALL Copy(uint64_t c,uint64_t id,HIGGS_HandPartSetState* p){if(!p)return HIGGS_HAND_INVALID_ARGUMENT;if(!Sized(p))return HIGGS_HAND_BUFFER_TOO_SMALL;return Guard([&]{HIGGS_HandPartSetState out{};auto s=HiggsHands::HandParts().Copy(c,id,out);if(!s)*p=out;return s;});}
HIGGS_HandStatus HIGGS_HAND_CALL Resolve(uint64_t c,uint64_t e,uint32_t key,uint64_t gen,HIGGS_HandPartAttribution* p){if(!p)return HIGGS_HAND_INVALID_ARGUMENT;if(!Sized(p))return HIGGS_HAND_BUFFER_TOO_SMALL;return Guard([&]{HIGGS_HandPartAttribution out{};auto s=HiggsHands::HandParts().Resolve(c,e,key,gen,out);if(!s)*p=out;return s;});}
HIGGS_HandStatus HIGGS_HAND_CALL Contact(uint64_t c,uint64_t e,uint64_t seq,HIGGS_HandPartContact* p){if(!p)return HIGGS_HAND_INVALID_ARGUMENT;if(!Sized(p))return HIGGS_HAND_BUFFER_TOO_SMALL;return Guard([&]{HIGGS_HandPartContact out{};auto s=HiggsHands::HandParts().CopyContact(c,e,seq,out);if(!s)*p=out;return s;});}
HIGGS_HandStatus HIGGS_HAND_CALL NativeResolve(uint64_t key,HIGGS_HandPartAttribution* p){if(!key||!p)return HIGGS_HAND_INVALID_ARGUMENT;if(!Sized(p))return HIGGS_HAND_BUFFER_TOO_SMALL;return Guard([&]{const auto resolver=nativeResolver.load();if(!resolver)return HIGGS_HandStatus(HIGGS_HAND_NOT_READY);HIGGS_HandPartAttribution value{};const auto status=resolver(key,value);if(!status)*p=value;return status;});}
}
namespace HiggsHands {void InstallHandPartNativeResolver(HandPartNativeResolver resolver){nativeResolver.store(resolver);}}
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandPartsInterface(uint32_t v,uint32_t n,HIGGS_HandPartsInterface* p){if(!p)return HIGGS_HAND_INVALID_ARGUMENT;if(v!=1)return HIGGS_HAND_UNSUPPORTED_VERSION;if(n<sizeof(*p))return HIGGS_HAND_BUFFER_TOO_SMALL;return Guard([&]{*p={sizeof(*p),1,HiggsHands::HandParts().Capabilities(),Sources,Set,End,Copy,Resolve,Contact};return HIGGS_HAND_OK;});}
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandPartsNativeInterface(uint32_t v,uint32_t n,HIGGS_HandPartsNativeInterface* p){if(!p)return HIGGS_HAND_INVALID_ARGUMENT;if(v!=1)return HIGGS_HAND_UNSUPPORTED_VERSION;if(n<sizeof(*p))return HIGGS_HAND_BUFFER_TOO_SMALL;*p={sizeof(*p),1,nativeResolver.load()?UINT64_C(1):UINT64_C(0),NativeResolve};return HIGGS_HAND_OK;}
