#include "hand_abi_boundary.h"
#include "hand_observations.h"
#include "hand_broker.h"
#include <cstring>
namespace {

HIGGS_HandStatus HIGGS_HAND_CALL Motion(uint64_t client,const HIGGS_ObservationQuery* query,HIGGS_MotionSample* out,uint32_t cap,uint32_t* count){
    if(!query||!count)return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::Guard([&]()->HIGGS_HandStatus{if(!HiggsHands::Service().ClientValid(client))return HIGGS_HAND_INVALID_CLIENT;
        const auto epoch=HiggsHands::Service().Epoch();HiggsHands::Observations().Epoch(epoch);
        if(query->epoch!=epoch)return HIGGS_HAND_STALE;
        return HiggsHands::Observations().CopyMotion(*query,out,cap,*count);});
}
HIGGS_HandStatus HIGGS_HAND_CALL Contacts(uint64_t client,const HIGGS_ObservationQuery* query,HIGGS_ContactObservation* out,uint32_t cap,uint32_t* count){
    if(!query||!count)return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::Guard([&]()->HIGGS_HandStatus{if(!HiggsHands::Service().ClientValid(client))return HIGGS_HAND_INVALID_CLIENT;
        const auto epoch=HiggsHands::Service().Epoch();HiggsHands::Observations().Epoch(epoch);
        if(query->epoch!=epoch)return HIGGS_HAND_STALE;
        return HiggsHands::Observations().CopyContacts(*query,out,cap,*count);});
}
HIGGS_HandStatus HIGGS_HAND_CALL Release(uint64_t client,uint64_t id,HIGGS_ReleaseObservation* out){
    if(!out)return HIGGS_HAND_INVALID_ARGUMENT;
    if(out->structSize<sizeof(*out)){out->structSize=sizeof(*out);return HIGGS_HAND_BUFFER_TOO_SMALL;}
    return HiggsAbi::Guard([&]()->HIGGS_HandStatus{if(!HiggsHands::Service().ClientValid(client))return HIGGS_HAND_INVALID_CLIENT;
        const auto epoch=HiggsHands::Service().Epoch();HiggsHands::Observations().Epoch(epoch);
        HIGGS_ReleaseObservation value{};auto status=HiggsHands::Observations().CopyRelease(epoch,id,value);
        if(!status)*out=value;return status;});
}
HIGGS_HandStatus HIGGS_HAND_CALL Mappings(uint64_t client,const HIGGS_ObservationQuery* query,HIGGS_TrackingMappingSample* out,uint32_t cap,uint32_t* count){
    if(!query||!count)return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::Guard([&]()->HIGGS_HandStatus{if(!HiggsHands::Service().ClientValid(client))return HIGGS_HAND_INVALID_CLIENT;
        const auto epoch=HiggsHands::Service().Epoch();HiggsHands::Observations().Epoch(epoch);
        if(query->epoch!=epoch)return HIGGS_HAND_STALE;
        return HiggsHands::Observations().CopyMappings(*query,out,cap,*count);});
}
}
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryObservationInterface(uint32_t version,uint32_t size,HIGGS_ObservationAPI* out){
    if(version!=HIGGS_OBSERVATION_ABI_VERSION)return HIGGS_HAND_UNSUPPORTED_VERSION;
    if(!out)return HIGGS_HAND_INVALID_ARGUMENT;
    if(size<sizeof(*out))return HIGGS_HAND_BUFFER_TOO_SMALL;
    constexpr auto capabilities=HIGGS_OBSERVATION_CAP_MOTION|HIGGS_OBSERVATION_CAP_CONTACT|HIGGS_OBSERVATION_CAP_RELEASE|HIGGS_OBSERVATION_CAP_CONTROLLER_MAPPING;
    const HIGGS_ObservationAPI table{sizeof(table),HIGGS_OBSERVATION_ABI_VERSION,capabilities,Motion,Contacts,Release,Mappings};std::memcpy(out,&table,sizeof(table));return HIGGS_HAND_OK;
}
namespace {
HIGGS_HandStatus HIGGS_HAND_CALL ContactDetails(uint64_t owner,uint64_t epoch,uint64_t seq,HIGGS_ContactDetails* out){
    if(!out)return HIGGS_HAND_INVALID_ARGUMENT;
    if(out->structSize<sizeof(*out))return HIGGS_HAND_BUFFER_TOO_SMALL;
    return HiggsAbi::Guard([&]()->HIGGS_HandStatus{
        if(!HiggsHands::Service().ClientValid(owner))return HIGGS_HAND_INVALID_CLIENT;
        if(epoch!=HiggsHands::Service().Epoch())return HIGGS_HAND_STALE;
        return HiggsHands::Observations().CopyContactDetails(epoch,seq,*out);
    });
}
}
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryContactDetailsInterface(uint32_t v,uint32_t size,HIGGS_ContactDetailsInterface* out){
    if(v!=1)return HIGGS_HAND_UNSUPPORTED_VERSION;
    if(!out)return HIGGS_HAND_INVALID_ARGUMENT;
    if(size<sizeof(*out))return HIGGS_HAND_BUFFER_TOO_SMALL;
    *out={sizeof(*out),1,ContactDetails};return HIGGS_HAND_OK;
}
