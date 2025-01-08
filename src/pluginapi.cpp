#include <vector>

#include "pluginapi.h"
#include "version.h"
#include "papyrusapi.h"
#include "config.h"
#include "hand.h"

using namespace HiggsPluginAPI;

// A message used to fetch HIGGS's interface
struct HiggsMessage {
    enum { kMessage_GetInterface = 0xF9279A57 }; // Randomly generated
    void * (*getApiFunction)(unsigned int revisionNumber) = nullptr;
};

// Interface classes are stored statically
HiggsInterface001 g_interface001;

// Constructs and returns an API of the revision number requested
void * GetApi(unsigned int revisionNumber) {
    switch (revisionNumber) {
    case 1:	_MESSAGE("Interface revision 1 requested"); return &g_interface001;
    }
    return nullptr;
}

// Handles skse mod messages requesting to fetch API functions from VRIK
void HiggsPluginAPI::ModMessageHandler(SKSEMessagingInterface::Message * message) {
    if (message->type == HiggsMessage::kMessage_GetInterface) {
        HiggsMessage * higgsMessage = (HiggsMessage*)message->data;
        higgsMessage->getApiFunction = GetApi;
        _MESSAGE("Provided HIGGS plugin interface to \"%s\"", message->sender);
    }
}

// HIGGS build numbers are made up as follows: V01.00.05.00
constexpr int higgsBuildNumber = FPVR_VERSION_MAJOR * 1000000 + FPVR_VERSION_MINOR * 10000 + FPVR_VERSION_PATCH * 100 + FPVR_VERSION_BETA;

// Fetches the HIGGS version number
unsigned int HiggsInterface001::GetBuildNumber() {
    return higgsBuildNumber;
}

bool HiggsInterface001::GetSettingDouble(const std::string_view& name, double& out) {
    return Config::GetSettingDouble(name, out);
}

bool HiggsInterface001::SetSettingDouble(const std::string& name, double val) {
    return Config::SetSettingDouble(name, val);
}

void HiggsInterface001::AddPulledCallback(PulledCallback callback) {
    if (!callback) return;
    std::scoped_lock lock(addCallbackLock);
    pulledCallbacks.push_back(callback);
}

void HiggsInterface001::AddGrabbedCallback(GrabbedCallback callback) {
    if (!callback) return;
    std::scoped_lock lock(addCallbackLock);
    grabbedCallbacks.push_back(callback);
}

void HiggsInterface001::AddDroppedCallback(DroppedCallback callback) {
    if (!callback) return;
    std::scoped_lock lock(addCallbackLock);
    droppedCallbacks.push_back(callback);
}

void HiggsInterface001::AddStashedCallback(StashedCallback callback) {
    if (!callback) return;
    std::scoped_lock lock(addCallbackLock);
    stashedCallbacks.push_back(callback);
}

void HiggsInterface001::AddConsumedCallback(ConsumedCallback callback) {
    if (!callback) return;
    std::scoped_lock lock(addCallbackLock);
    consumedCallbacks.push_back(callback);
}

void HiggsInterface001::AddCollisionCallback(CollisionCallback callback) {
    if (!callback) return;
    std::scoped_lock lock(addCallbackLock);
    collisionCallbacks.push_back(callback);
}

void HiggsInterface001::AddStartTwoHandingCallback(StartTwoHandingCallback callback) {
    if (!callback) return;
    std::scoped_lock lock(addCallbackLock);
    startTwoHandingCallbacks.push_back(callback);
}

void HiggsInterface001::AddStopTwoHandingCallback(StopTwoHandingCallback callback) {
    if (!callback) return;
    std::scoped_lock lock(addCallbackLock);
    stopTwoHandingCallbacks.push_back(callback);
}

void HiggsInterface001::AddCollisionFilterComparisonCallback(CollisionFilterComparisonCallback callback) {
    if (!callback) return;
    std::scoped_lock lock(addCallbackLock);
    collisionFilterComparisonCallbacks.push_back(callback);
}

void HiggsInterface001::AddPrePhysicsStepCallback(PrePhysicsStepCallback callback) {
    if (!callback) return;
    std::scoped_lock lock(addCallbackLock);
    prePhysicsStepCallbacks.push_back(callback);
}

void HiggsInterface001::AddPreVrikPreHiggsCallback(NoArgCallback callback) {
    if (!callback) return;
    std::scoped_lock lock(addCallbackLock);
    preVrikPreHiggsCallbacks.push_back(callback);
}

void HiggsInterface001::AddPreVrikPostHiggsCallback(NoArgCallback callback) {
    if (!callback) return;
    std::scoped_lock lock(addCallbackLock);
    preVrikPostHiggsCallbacks.push_back(callback);
}

void HiggsInterface001::AddPostVrikPreHiggsCallback(NoArgCallback callback) {
    if (!callback) return;
    std::scoped_lock lock(addCallbackLock);
    postVrikPreHiggsCallbacks.push_back(callback);
}

void HiggsInterface001::AddPostVrikPostHiggsCallback(NoArgCallback callback) {
    if (!callback) return;
    std::scoped_lock lock(addCallbackLock);
    postVrikPostHiggsCallbacks.push_back(callback);
}


void HiggsPluginAPI::TriggerPulledCallbacks(bool isLeft, TESObjectREFR *pulledRefr) {
    for (auto callback : g_interface001.pulledCallbacks) {
        callback(isLeft, pulledRefr);
    }

    PapyrusAPI::OnPullEvent(pulledRefr, isLeft);
}

void HiggsPluginAPI::TriggerGrabbedCallbacks(bool isLeft, TESObjectREFR *grabbedRefr) {
    for (auto callback : g_interface001.grabbedCallbacks) {
        callback(isLeft, grabbedRefr);
    }

    PapyrusAPI::OnGrabEvent(grabbedRefr, isLeft);
}

void HiggsPluginAPI::TriggerDroppedCallbacks(bool isLeft, TESObjectREFR *droppedRefr) {
    for (auto callback : g_interface001.droppedCallbacks) {
        callback(isLeft, droppedRefr);
    }

    PapyrusAPI::OnDropEvent(droppedRefr, isLeft);
}

void HiggsPluginAPI::TriggerStashedCallbacks(bool isLeft, TESForm *stashedForm) {
    for (auto callback : g_interface001.stashedCallbacks) {
        callback(isLeft, stashedForm);
    }

    PapyrusAPI::OnStashEvent(stashedForm, isLeft);
}

void HiggsPluginAPI::TriggerConsumedCallbacks(bool isLeft, TESForm *consumedForm) {
    for (auto callback : g_interface001.consumedCallbacks) {
        callback(isLeft, consumedForm);
    }

    PapyrusAPI::OnConsumeEvent(consumedForm, isLeft);
}

void HiggsPluginAPI::TriggerCollisionCallbacks(bool isLeft, float mass, float separatingVelocity) {
    for (auto callback : g_interface001.collisionCallbacks) {
        callback(isLeft, mass, separatingVelocity);
    }
}

void HiggsPluginAPI::TriggerStartTwoHandingCallbacks() {
    for (auto callback : g_interface001.startTwoHandingCallbacks) {
        callback();
    }

    PapyrusAPI::OnStartTwoHandingEvent();
}

void HiggsPluginAPI::TriggerStopTwoHandingCallbacks() {
    for (auto callback : g_interface001.stopTwoHandingCallbacks) {
        callback();
    }

    PapyrusAPI::OnStopTwoHandingEvent();
}

CollisionFilterComparisonResult HiggsPluginAPI::TriggerCollisionFilterComparisonCallbacks(void *filter, UInt32 filterInfoA, UInt32 filterInfoB) {
    for (auto callback : g_interface001.collisionFilterComparisonCallbacks) {
        CollisionFilterComparisonResult result = callback(filter, filterInfoA, filterInfoB);
        if (result == CollisionFilterComparisonResult::Collide || result == CollisionFilterComparisonResult::Ignore) {
            return result;
        }
        // result is CollisionFilterComparisonResult::Continue, i.e. don't do anything
    }

    return CollisionFilterComparisonResult::Continue;
}

void HiggsPluginAPI::TriggerPrePhysicsStepCallbacks(void *world) {
    for (auto callback : g_interface001.prePhysicsStepCallbacks) {
        callback(world);
    }
}

void HiggsPluginAPI::TriggerPreVrikPreHiggsCallbacks() {
    for (auto callback : g_interface001.preVrikPreHiggsCallbacks) {
        callback();
    }
}

void HiggsPluginAPI::TriggerPreVrikPostHiggsCallbacks() {
    for (auto callback : g_interface001.preVrikPostHiggsCallbacks) {
        callback();
    }
}

void HiggsPluginAPI::TriggerPostVrikPreHiggsCallbacks() {
    for (auto callback : g_interface001.postVrikPreHiggsCallbacks) {
        callback();
    }
}

void HiggsPluginAPI::TriggerPostVrikPostHiggsCallbacks() {
    for (auto callback : g_interface001.postVrikPostHiggsCallbacks) {
        callback();
    }
}

void HiggsInterface001::GrabObject(TESObjectREFR *object, bool isLeft)
{
    Hand &hand = isLeft ? *g_leftHand : *g_rightHand;

    hand.externalGrabRequestedObject = object;
    hand.externalGrabRequested = true;
}

TESObjectREFR * HiggsInterface001::GetGrabbedObject(bool isLeft)
{
    Hand &hand = isLeft ? *g_leftHand : *g_rightHand;

    if (hand.HasHeldObject()) {
        UInt32 handle = hand.selectedObject.handle;
        // To be somewhat thread-safe, check again if we're in held after getting the handle
        if (hand.HasHeldObject()) {
            NiPointer<TESObjectREFR> grabbedObj;
            if (LookupREFRByHandle(handle, grabbedObj)) {
                return grabbedObj;
            }
        }
    }

    return nullptr;
}

bool HiggsInterface001::IsHandInGrabbableState(bool isLeft)
{
    Hand &hand = isLeft ? *g_leftHand : *g_rightHand;
    return hand.IsInGrabbableState();
}

void HiggsInterface001::DisableHand(bool isLeft)
{
    if (isLeft) {
        isLeftHandDisabled = true;
    }
    else {
        isRightHandDisabled = true;
    }
}

void HiggsInterface001::EnableHand(bool isLeft)
{
    if (isLeft) {
        isLeftHandDisabled = false;
    }
    else {
        isRightHandDisabled = false;
    }
}

bool HiggsInterface001::IsDisabled(bool isLeft)
{
    if (isLeft) {
        return isLeftHandDisabled;
    }
    else {
        return isRightHandDisabled;
    }
}

void HiggsInterface001::DisableWeaponCollision(bool isLeft)
{
    if (isLeft) {
        isLeftWeaponCollisionDisabled = true;
    }
    else {
        isRightWeaponCollisionDisabled = true;
    }
}

void HiggsInterface001::EnableWeaponCollision(bool isLeft)
{
    if (isLeft) {
        isLeftWeaponCollisionDisabled = false;
    }
    else {
        isRightWeaponCollisionDisabled = false;
    }
}

bool HiggsInterface001::IsWeaponCollisionDisabled(bool isLeft)
{
    if (isLeft) {
        return isLeftWeaponCollisionDisabled;
    }
    else {
        return isRightWeaponCollisionDisabled;
    }
}

void HiggsInterface001::ForceWeaponCollisionEnabled(bool isLeft)
{
    forceEnableWeaponCollision[isLeft] = true;
}

bool HiggsInterface001::IsTwoHanding()
{
    return g_rightHand->IsTwoHanding() || g_leftHand->IsTwoHanding();
}

bool HiggsInterface001::CanGrabObject(bool isLeft)
{
    Hand &hand = isLeft ? *g_leftHand : *g_rightHand;
    return hand.CanGrabObject();
}

NiObject * HiggsInterface001::GetHandRigidBody(bool isLeft)
{
    Hand &hand = isLeft ? *g_leftHand : *g_rightHand;
    return hand.handBody;
}

NiObject * HiggsInterface001::GetWeaponRigidBody(bool isLeft)
{
    Hand &hand = isLeft ? *g_leftHand : *g_rightHand;
    return hand.weaponBody;
}

NiObject * HiggsInterface001::GetGrabbedRigidBody(bool isLeft)
{
    Hand &hand = isLeft ? *g_leftHand : *g_rightHand;
    if (hand.HasHeldObject()) {
        return hand.selectedObject.rigidBody;
    }
    return nullptr;
}

UInt64 HiggsInterface001::GetHiggsLayerBitfield()
{
    return higgsLayerBitfield;
}

void HiggsInterface001::SetHiggsLayerBitfield(UInt64 bitfield)
{
    higgsLayerBitfield = bitfield;
}

NiTransform HiggsInterface001::GetGrabTransform(bool isLeft)
{
    Hand &hand = isLeft ? *g_leftHand : *g_rightHand;
    return hand.GetGrabTransform();
}

void HiggsInterface001::SetGrabTransform(bool isLeft, const NiTransform &transform)
{
    Hand &hand = isLeft ? *g_leftHand : *g_rightHand;
    hand.SetGrabTransform(transform);
}

bool HiggsInterface001::IsHoldingObject(bool isLeft)
{
    Hand &hand = isLeft ? *g_leftHand : *g_rightHand;
    return hand.HasHeldObject();
}

void HiggsInterface001::GetFingerValues(bool isLeft, float values[5])
{
    Hand &hand = isLeft ? *g_leftHand : *g_rightHand;
    for (int i = 0; i < 5; i++) {
        values[i] = hand.grabbedFingerValues[i];
    }
}
