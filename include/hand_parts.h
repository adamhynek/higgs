#pragma once
#include "higgsphysics.h"
#include <array>
#include <mutex>
namespace HiggsHands {
class Broker;
class HandPartService {
public:
    explicit HandPartService(Broker& b):broker_(b){}
    using Driver=HIGGS_HandStatus(*)(const HIGGS_HandPartSetRequest&,const HIGGS_HandPartSources&,
                                    HIGGS_HandPartSetState&,bool cleanup,void*);
    void Install(Driver,void*,uint64_t);
    uint64_t Capabilities();
    bool Publish(const HIGGS_HandPartSources&);
    HIGGS_HandStatus Sources(uint64_t,uint32_t,HIGGS_HandPartSources&,uint64_t);
    HIGGS_HandStatus Set(uint64_t,const HIGGS_HandPartSetRequest&,HIGGS_HandPartSetState&,uint64_t);
    HIGGS_HandStatus End(uint64_t,uint64_t,uint64_t);
    HIGGS_HandStatus Copy(uint64_t,uint64_t,HIGGS_HandPartSetState&);
    HIGGS_HandStatus Resolve(uint64_t,uint64_t,uint32_t,uint64_t,HIGGS_HandPartAttribution&);
    bool Contact(const HIGGS_HandPartContact&);
    HIGGS_HandStatus CopyContact(uint64_t,uint64_t,uint64_t,HIGGS_HandPartContact&);
    void Drain(uint64_t);
    void Invalidate(uint32_t reason);
private:
    struct Slot {HIGGS_HandPartSetState state{};HIGGS_HandPartSetRequest request{};};
    Slot* Find(uint64_t);
    Broker& broker_;
    std::mutex mutex_;
    std::array<HIGGS_HandPartSources,2> sources_{};
    std::array<Slot,32> slots_{};
    std::array<HIGGS_HandPartContact,512> contacts_{};
    size_t contactCount_=0,contactWrite_=0;
    uint64_t contactEpoch_=0;
    uint64_t next_=1,capabilities_=0;
    Driver driver_=nullptr;void* context_=nullptr;
};
HandPartService& HandParts();
using HandPartNativeResolver=HIGGS_HandStatus(*)(uint64_t,HIGGS_HandPartAttribution&);
void InstallHandPartNativeResolver(HandPartNativeResolver);
bool ValidHandPartSource(const HIGGS_HandPartSources&);
bool HandPartTarget(const HIGGS_HandPartSources&,const HIGGS_HandPartDefinition&,float*,float*);
}
