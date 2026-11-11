#pragma once
#include "higgsobservations.h"
#include <array>
#include <mutex>
#define HIGGS_OBSERVATIONS_NATIVE 1

class hkpContactPointEvent;
class NiAVObject;
namespace vr_1_0_12 { struct TrackedDevicePose_t; }
namespace HiggsHands {
class ObservationStore {
public:
    bool Motion(HIGGS_MotionSample);
    bool Contact(HIGGS_ContactObservation,uint64_t* assignedSequence=nullptr, HIGGS_ContactDetails details={});
    HIGGS_HandStatus CopyContactDetails(uint64_t epoch,uint64_t sequence,HIGGS_ContactDetails&);
    bool Controllers(const std::array<HIGGS_MotionSample,4>&,const HIGGS_TrackingMappingSample*);
    void Release(const HIGGS_HandEvent&,const HIGGS_HandSnapshot&);
    void Epoch(uint64_t);
    HIGGS_HandStatus CopyMotion(const HIGGS_ObservationQuery&,HIGGS_MotionSample*,uint32_t,uint32_t&);
    HIGGS_HandStatus CopyContacts(const HIGGS_ObservationQuery&,HIGGS_ContactObservation*,uint32_t,uint32_t&);
    HIGGS_HandStatus CopyRelease(uint64_t,uint64_t,HIGGS_ReleaseObservation&);
    HIGGS_HandStatus CopyMappings(const HIGGS_ObservationQuery&,HIGGS_TrackingMappingSample*,uint32_t,uint32_t&);
private:
    void EpochLocked(uint64_t);
    std::mutex mutex_;
    uint64_t epoch_=0,next_=1;
    uint64_t lastTrackingGeneration_=0,lastCapture_=0;
    std::array<HIGGS_MotionSample,256> motion_{};
    std::array<HIGGS_ContactObservation,512> contacts_{};
    std::array<HIGGS_ContactDetails,512> details_{};
    std::array<HIGGS_ReleaseObservation,64> releases_{};
    std::array<HIGGS_TrackingMappingSample,64> mappings_{};
    size_t motionCount_=0,contactCount_=0,releaseCount_=0;
    size_t motionWrite_=0,contactWrite_=0,releaseWrite_=0,mappingCount_=0,mappingWrite_=0;
};
ObservationStore& Observations();
void ObserveRelease(const HIGGS_HandEvent&,const HIGGS_HandSnapshot&);
void ObserveNativeSamples();
void ObserveNativeContact(const hkpContactPointEvent&,uint32_t hands,uint32_t source,bool probeIsA);
void ObserveNativeControllers(NiAVObject*,const vr_1_0_12::TrackedDevicePose_t*,uint32_t,
    uint64_t trackingGeneration,uint64_t captureId,uint64_t observedAtNs);
}
