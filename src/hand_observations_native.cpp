#include "hand_observations.h"
#include "hand_broker.h"
#include "hand_parts_native.h"
#include "RE/offsets.h"
#include "hand.h"
#include "utils.h"
#include "main.h"
#include "controller_tracking.h"
#include "skse64/GameVR.h"
#include <cstring>
#include <cmath>

namespace HiggsHands {
namespace {
void Vector(float out[3],const hkVector4& v,float scale=1.f){out[0]=v(0)*scale;out[1]=v(1)*scale;out[2]=v(2)*scale;}
}
void ObserveNativeSamples(){
    const auto snapshot=Service().CopyPublished();Observations().Epoch(snapshot.epoch);
    if(!snapshot.worldReady)return;
    for(const auto& hand:snapshot.hands){
        if(!(hand.observationMask&HIGGS_OBS_PALM_VALID))continue;
        HIGGS_MotionSample sample{};sample.structSize=sizeof(sample);sample.physicalHand=hand.physicalHand;
        sample.source=HIGGS_SAMPLE_PALM;sample.space=HIGGS_SPACE_WORLD;
        sample.flags=HIGGS_SAMPLE_POSE|HIGGS_SAMPLE_NO_HARDWARE_TIME;sample.epoch=snapshot.epoch;
        sample.observationId=snapshot.observationId;sample.handGeneration=hand.generation;sample.observedAtNs=hand.poseObservedAtNs;
        std::memcpy(sample.position,hand.palmPosition,sizeof(sample.position));std::memcpy(sample.rotation,hand.palmRotation,sizeof(sample.rotation));
        Observations().Motion(sample);
        auto* native=hand.physicalHand==HIGGS_HAND_LEFT?g_leftHand:g_rightHand;
        NiPointer<bhkRigidBody> wrapper=native&&native->HasHeldObject()?native->selectedObject.rigidBody:nullptr;
        if(!wrapper||!wrapper->hkBody||!hand.heldRefId)continue;
        RE::hkRefPtr<hkpRigidBody> body=wrapper->hkBody;
        auto* player=*g_thePlayer;
        NiPointer<bhkWorld> world=player&&player->parentCell?GetHavokWorldFromCell(player->parentCell):nullptr;
        if(!world||!world->world)continue;
        BSReadLocker worldLock(&world->worldLock);if(body->getWorld()!=world->world.val())continue;
        sample.source=HIGGS_SAMPLE_HELD_BODY;sample.itemRefId=hand.heldRefId;sample.bodyKey=body->m_uid;
        sample.flags|=HIGGS_SAMPLE_LINEAR|HIGGS_SAMPLE_ANGULAR;
        Vector(sample.position,body->getPosition(),*g_inverseHavokWorldScale);
        for(int i=0;i<3;++i)for(int j=0;j<3;++j)sample.rotation[i*3+j]=body->getTransform().getRotation().getColumn(j)(i);
        Vector(sample.linearVelocity,body->getLinearVelocity());Vector(sample.angularVelocity,body->getAngularVelocity());
        sample.observedAtNs=Now();Observations().Motion(sample);
    }
}
void ObserveNativeContact(const hkpContactPointEvent& event,uint32_t hands,uint32_t source,bool probeIsA){
    if(!event.m_contactPoint||!event.m_contactPointProperties||!event.m_bodies[0]||!event.m_bodies[1]||(event.m_contactPointProperties->m_flags&hkContactPointMaterial::FlagEnum::CONTACT_IS_DISABLED))return;
    const auto snapshot=Service().CopyPublished();if(!snapshot.worldReady)return;
    const auto* probe=event.m_bodies[probeIsA?0:1];const auto* target=event.m_bodies[probeIsA?1:0];
    HIGGS_ContactObservation value{};value.structSize=sizeof(value);value.physicalHands=hands;value.source=source;
    value.flags=HIGGS_CONTACT_ENABLED|HIGGS_CONTACT_POINT|HIGGS_CONTACT_NORMAL|HIGGS_CONTACT_LINEAR;
    if(event.m_contactPointProperties->wasUsed())value.flags|=HIGGS_CONTACT_USED;
    if(event.m_contactPointProperties->isPotential())value.flags|=HIGGS_CONTACT_POTENTIAL;
    value.epoch=snapshot.epoch;value.observationId=snapshot.observationId;value.observedAtNs=Now();
    value.probeBodyKey=probe->m_uid;value.targetBodyKey=target->m_uid;
    Vector(value.position,event.m_contactPoint->getPosition(),*g_inverseHavokWorldScale);
    Vector(value.targetToProbeNormal,event.m_contactPoint->getNormal(),probeIsA?1.f:-1.f);
    Vector(value.probeLinearVelocity,probe->getLinearVelocity());Vector(value.targetLinearVelocity,target->getLinearVelocity());
    if(auto ref=GetRefFromCollidable(target->getCollidable())){value.targetRefId=ref->formID;value.targetFormType=ref->baseForm?ref->baseForm->formType:0;}
    if(auto ref=GetRefFromCollidable(probe->getCollidable()))value.itemRefId=ref->formID;
    uint64_t sequence=0;
    if(Observations().Contact(value,&sequence))RecordNativeHandPartContact(probe,value.epoch,sequence);
}
namespace {
bool PoseFinite(const vr_1_0_12::HmdMatrix34_t& m){for(const auto& row:m.m)for(float value:row)if(!std::isfinite(value))return false;return true;}
bool TransformFinite(const NiTransform& t){
    if(!std::isfinite(t.pos.x)||!std::isfinite(t.pos.y)||!std::isfinite(t.pos.z)||!std::isfinite(t.scale)||t.scale<=0)return false;
    HiggsMotion::Rotation rotation{};for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)rotation[i][j]=t.rot.data[i][j];return HiggsMotion::IsRotation(rotation);
}
void Pose(HIGGS_MotionSample& sample,const NiTransform& t){sample.position[0]=t.pos.x;sample.position[1]=t.pos.y;sample.position[2]=t.pos.z;for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)sample.rotation[i*3+j]=t.rot.data[i][j];}
bool CaptureMapping(NiAVObject* hmd,const vr_1_0_12::TrackedDevicePose_t& rawHmd,NiTransform& worldFromConverted,HIGGS_TrackingMappingSample& mapping){
    if(!hmd||!rawHmd.bDeviceIsConnected||!rawHmd.bPoseIsValid||rawHmd.eTrackingResult!=vr_1_0_12::TrackingResult_Running_OK||!PoseFinite(rawHmd.mDeviceToAbsoluteTracking)||!TransformFinite(hmd->m_worldTransform))return false;
    NiTransform convertedHmd;HmdMatrixToNiTransform(convertedHmd,rawHmd.mDeviceToAbsoluteTracking);if(!TransformFinite(convertedHmd))return false;NiTransform inverse;convertedHmd.Invert(inverse);worldFromConverted=hmd->m_worldTransform*inverse;if(!TransformFinite(worldFromConverted))return false;
    vr_1_0_12::HmdMatrix34_t basis{};for(unsigned i=0;i<3;++i)basis.m[i][i]=1;
    NiTransform convertedOrigin;HmdMatrixToNiTransform(convertedOrigin,basis);if(!TransformFinite(convertedOrigin))return false;const auto origin=worldFromConverted*convertedOrigin.pos;
    mapping.translation[0]=origin.x;mapping.translation[1]=origin.y;mapping.translation[2]=origin.z;
    double lengths[3]{};
    for(unsigned axis=0;axis<3;++axis){basis.m[axis][3]=1;NiTransform converted;HmdMatrixToNiTransform(converted,basis);basis.m[axis][3]=0;if(!TransformFinite(converted))return false;const auto point=worldFromConverted*converted.pos;const float delta[3]{point.x-origin.x,point.y-origin.y,point.z-origin.z};for(unsigned row=0;row<3;++row){mapping.rotation[row*3+axis]=delta[row];lengths[axis]+=double(delta[row])*delta[row];}lengths[axis]=std::sqrt(lengths[axis]);}
    const double scale=(lengths[0]+lengths[1]+lengths[2])/3.;if(!std::isfinite(scale)||scale<=0)return false;
    for(unsigned axis=0;axis<3;++axis){if(std::abs(lengths[axis]-scale)>scale*.001)return false;for(unsigned row=0;row<3;++row)mapping.rotation[row*3+axis]=static_cast<float>(mapping.rotation[row*3+axis]/scale);}
    HiggsMotion::Rotation rotation{};for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)rotation[i][j]=mapping.rotation[i*3+j];if(!HiggsMotion::IsRotation(rotation))return false;mapping.unitsPerMeter=static_cast<float>(scale);return std::isfinite(mapping.unitsPerMeter)&&mapping.unitsPerMeter>0;
}
}
void ObserveNativeControllers(NiAVObject* hmd,const vr_1_0_12::TrackedDevicePose_t* poses,uint32_t count,uint64_t trackingGeneration,uint64_t captureId,uint64_t observedAtNs){
    try{
        const auto tracking=HiggsMotion::Tracking().CopyObservation();
        if(!trackingGeneration||!captureId||!observedAtNs||tracking.generation!=trackingGeneration||tracking.lastAcceptedObservation!=captureId||tracking.loading||!tracking.worldObserved||tracking.exhausted)return;
        const auto snapshot=Service().CopyPublished();Observations().Epoch(snapshot.epoch);if(!snapshot.worldReady)return;
        std::array<HIGGS_MotionSample,4> samples{};
        for(unsigned i=0;i<samples.size();++i){auto& sample=samples[i];sample.structSize=sizeof(sample);sample.physicalHand=i/2+1;sample.source=HIGGS_SAMPLE_CONTROLLER;sample.space=i%2?HIGGS_SPACE_WORLD:HIGGS_SPACE_OPENVR_TRACKING;sample.flags=HIGGS_SAMPLE_NO_HARDWARE_TIME;sample.status=HIGGS_SAMPLE_UNAVAILABLE;sample.deviceIndex=UINT32_MAX;sample.epoch=snapshot.epoch;sample.observationId=snapshot.observationId;sample.handGeneration=snapshot.hands[i/2].generation;sample.observedAtNs=observedAtNs;sample.trackingGeneration=trackingGeneration;sample.captureId=captureId;}
        auto* system=g_openVR&&*g_openVR?(*g_openVR)->vrSystem:nullptr;
        HIGGS_TrackingMappingSample mapping{};mapping.structSize=sizeof(mapping);mapping.flags=HIGGS_SAMPLE_NO_HARDWARE_TIME;mapping.epoch=snapshot.epoch;mapping.spaceGeneration=captureId;mapping.trackingGeneration=trackingGeneration;mapping.captureId=captureId;mapping.observedAtNs=observedAtNs;
        NiTransform worldFromConverted;bool mapped=false;
        if(system&&poses&&count>vr_1_0_12::k_unTrackedDeviceIndex_Hmd&&system->IsTrackedDeviceConnected(vr_1_0_12::k_unTrackedDeviceIndex_Hmd))mapped=CaptureMapping(hmd,poses[vr_1_0_12::k_unTrackedDeviceIndex_Hmd],worldFromConverted,mapping);
        uint32_t indices[2]{UINT32_MAX,UINT32_MAX};
        if(system){indices[0]=system->GetTrackedDeviceIndexForControllerRole(vr_1_0_12::TrackedControllerRole_LeftHand);indices[1]=system->GetTrackedDeviceIndexForControllerRole(vr_1_0_12::TrackedControllerRole_RightHand);}
        for(unsigned hand=0;hand<2;++hand){auto& raw=samples[hand*2];auto& world=samples[hand*2+1];raw.deviceIndex=world.deviceIndex=indices[hand];
            if(!system||!poses||indices[0]==indices[1])continue;
            const auto index=indices[hand];if(index==vr_1_0_12::k_unTrackedDeviceIndex_Hmd||index>=count||index>=vr_1_0_12::k_unMaxTrackedDeviceCount||!system->IsTrackedDeviceConnected(index)){raw.status=world.status=HIGGS_SAMPLE_DISCONNECTED;continue;}
            const auto& pose=poses[index];if(!pose.bDeviceIsConnected){raw.status=world.status=HIGGS_SAMPLE_DISCONNECTED;continue;}
            raw.status=world.status=HIGGS_SAMPLE_INVALID;if(!pose.bPoseIsValid||pose.eTrackingResult!=vr_1_0_12::TrackingResult_Running_OK||!PoseFinite(pose.mDeviceToAbsoluteTracking))continue;
            HiggsMotion::Rotation rawRotation{};for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)rawRotation[i][j]=pose.mDeviceToAbsoluteTracking.m[i][j];if(!HiggsMotion::IsRotation(rawRotation))continue;
            raw.status=HIGGS_SAMPLE_AVAILABLE;raw.flags|=HIGGS_SAMPLE_POSE;
            for(unsigned i=0;i<3;++i){raw.position[i]=pose.mDeviceToAbsoluteTracking.m[i][3];for(unsigned j=0;j<3;++j)raw.rotation[i*3+j]=pose.mDeviceToAbsoluteTracking.m[i][j];}
            bool linear=true,angular=true;for(unsigned i=0;i<3;++i){linear=linear&&std::isfinite(pose.vVelocity.v[i]);angular=angular&&std::isfinite(pose.vAngularVelocity.v[i]);}
            if(linear){raw.flags|=HIGGS_SAMPLE_LINEAR;for(unsigned i=0;i<3;++i)raw.linearVelocity[i]=pose.vVelocity.v[i];}if(angular){raw.flags|=HIGGS_SAMPLE_ANGULAR;for(unsigned i=0;i<3;++i)raw.angularVelocity[i]=pose.vAngularVelocity.v[i];}
            world.status=HIGGS_SAMPLE_UNAVAILABLE;
            if(mapped){world.status=HIGGS_SAMPLE_INVALID;NiTransform converted;HmdMatrixToNiTransform(converted,pose.mDeviceToAbsoluteTracking);if(!TransformFinite(converted))continue;const auto actual=worldFromConverted*converted;if(!TransformFinite(actual))continue;world.status=HIGGS_SAMPLE_AVAILABLE;world.flags|=HIGGS_SAMPLE_POSE|HIGGS_SAMPLE_MAPPING;world.spaceGeneration=mapping.spaceGeneration;raw.flags|=HIGGS_SAMPLE_MAPPING;raw.spaceGeneration=mapping.spaceGeneration;Pose(world,actual);
                const float metresPerTrackingMetre=mapping.unitsPerMeter * *g_havokWorldScale;
                if(linear&&std::isfinite(metresPerTrackingMetre)&&metresPerTrackingMetre>0){world.flags|=HIGGS_SAMPLE_LINEAR;for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)world.linearVelocity[i]+=mapping.rotation[i*3+j]*raw.linearVelocity[j]*metresPerTrackingMetre;}
                if(angular){world.flags|=HIGGS_SAMPLE_ANGULAR;for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)world.angularVelocity[i]+=mapping.rotation[i*3+j]*raw.angularVelocity[j];}
            }
        }
        const auto finalTracking=HiggsMotion::Tracking().CopyObservation();const auto current=Service().CopyPublished();
        if(finalTracking.generation!=trackingGeneration||finalTracking.lastAcceptedObservation!=captureId||current.epoch!=snapshot.epoch||current.hands[0].generation!=snapshot.hands[0].generation||current.hands[1].generation!=snapshot.hands[1].generation)return;
        Observations().Controllers(samples,mapped?&mapping:nullptr);
    }catch(...){/* A failed observation never escapes the VR pose callback. */}
}
}
