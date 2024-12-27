#include <functional>
#include <string>
#include <regex>
#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>
#include <limits>
#include <atomic>

#include "common/IDebugLog.h"  // IDebugLog
#include "skse64_common/skse_version.h"  // RUNTIME_VERSION
#include "skse64/GameRTTI.h"
#include "skse64/GameSettings.h"
#include "skse64/NiNodes.h"
#include "skse64/NiObjects.h"
#include "skse64/NiExtraData.h"
#include "skse64/GameData.h"
#include "skse64/GameForms.h"
#include "skse64/PapyrusActor.h"
#include "skse64/GameVR.h"
#include "skse64/gamethreads.h"
#include "skse64_common/SafeWrite.h"
#include "skse64_common/BranchTrampoline.h"

#include <ShlObj.h>  // CSIDL_MYDOCUMENTS

#include "RE/offsets.h"
#include "hand.h"
#include "version.h"
#include "utils.h"
#include "config.h"
#include "menu_checker.h"
#include "effects.h"
#include "hooks.h"
#include "vrikinterface001.h"
#include "papyrusapi.h"
#include "pluginapi.h"
#include "math_utils.h"
#include "physics.h"
#include "main.h"
#include "finger_curves.h"
#include "draw.h"

#include <Physics/Dynamics/World/Extensions/hkpWorldExtension.h>


// SKSE globals
static PluginHandle	g_pluginHandle = kPluginHandle_Invalid;
static SKSEMessagingInterface *g_messaging = nullptr;

SKSEVRInterface *g_vrInterface = nullptr;
SKSETrampolineInterface *g_trampoline = nullptr;
SKSEPapyrusInterface *g_papyrus = nullptr;
SKSETaskInterface *g_taskInterface = nullptr;

vrikPluginApi::IVrikInterface001 * g_vrikInterface;

TESEffectShader *g_itemSelectedShader = nullptr;
TESEffectShader *g_itemSelectedShaderOffLimits = nullptr;

bool initComplete = false; // Whether hands have been initialized

bool g_isVrikPresent = false;
SInt32 g_controllerType = BSOpenVR::ControllerTypes::kControllerType_Oculus;
bool g_isActivateBoundToGrip = false;

Hand *g_rightHand = nullptr;
Hand *g_leftHand = nullptr;

std::unordered_map<ShaderReferenceEffect *, std::unordered_set<BSGeometry *>> g_shaderNodes{};

PlayingShader g_playingShaders[2]{};
std::unordered_map<NiAVObject *, NiPointer<ShaderReferenceEffect>> g_effectDataMap{};

PhysicsListener g_physicsListener{};
IslandDeactivationListener g_activationListener;

int g_savedShadowUpdateFrameDelay = -1;
int g_shadowUpdateFrame = 0;
int g_numShadowUpdates = 0;


bool TryHook()
{
    // This should be sized to the actual amount used by your trampoline
    static const size_t TRAMPOLINE_SIZE = 1024;

    if (g_trampoline) {
        void* branch = g_trampoline->AllocateFromBranchPool(g_pluginHandle, TRAMPOLINE_SIZE);
        if (!branch) {
            _ERROR("couldn't acquire branch trampoline from SKSE. this is fatal. skipping remainder of init process.");
            return false;
        }

        g_branchTrampoline.SetBase(TRAMPOLINE_SIZE, branch);

        void* local = g_trampoline->AllocateFromLocalPool(g_pluginHandle, TRAMPOLINE_SIZE);
        if (!local) {
            _ERROR("couldn't acquire codegen buffer from SKSE. this is fatal. skipping remainder of init process.");
            return false;
        }

        g_localTrampoline.SetBase(TRAMPOLINE_SIZE, local);
    }
    else {
        if (!g_branchTrampoline.Create(TRAMPOLINE_SIZE)) {
            _ERROR("couldn't create branch trampoline. this is fatal. skipping remainder of init process.");
            return false;
        }
        if (!g_localTrampoline.Create(TRAMPOLINE_SIZE, nullptr))
        {
            _ERROR("couldn't create codegen buffer. this is fatal. skipping remainder of init process.");
            return false;
        }
    }

    PerformHooks();
    return true;
}


void FillControllerVelocities(NiAVObject *hmdNode, vr_src::TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount)
{
    if (g_openVR && *g_openVR) {
        BSOpenVR *openVR = *g_openVR;
        vr_src::IVRSystem *vrSystem = openVR->vrSystem;
        if (vrSystem) {
            const vr_src::TrackedDeviceIndex_t rightIndex = vrSystem->GetTrackedDeviceIndexForControllerRole(vr_src::ETrackedControllerRole::TrackedControllerRole_RightHand);
            const vr_src::TrackedDeviceIndex_t leftIndex = vrSystem->GetTrackedDeviceIndexForControllerRole(vr_src::ETrackedControllerRole::TrackedControllerRole_LeftHand);
            const vr_src::TrackedDeviceIndex_t hmdIndex = vr_src::k_unTrackedDeviceIndex_Hmd;

            if (unGamePoseArrayCount > hmdIndex && vrSystem->IsTrackedDeviceConnected(hmdIndex) && hmdNode) {
                vr_src::TrackedDevicePose_t &hmdPose = pGamePoseArray[hmdIndex];
                if (hmdPose.bDeviceIsConnected && hmdPose.bPoseIsValid && hmdPose.eTrackingResult == vr_src::ETrackingResult::TrackingResult_Running_OK) {
                    vr_src::HmdMatrix34_t &hmdMatrix = hmdPose.mDeviceToAbsoluteTracking;

                    NiTransform hmdTransform;
                    HmdMatrixToNiTransform(hmdTransform, hmdMatrix);

                    // Use the transform between the openvr hmd pose and skyrim's hmdnode transform to get the transform from openvr space to skyrim worldspace
                    NiMatrix33 openvrToSkyrimWorldTransform = hmdNode->m_worldTransform.rot * hmdTransform.rot.Transpose();

                    bool isRightConnected = vrSystem->IsTrackedDeviceConnected(rightIndex);
                    bool isLeftConnected = vrSystem->IsTrackedDeviceConnected(leftIndex);

                    for (int i = hmdIndex + 1; i < unGamePoseArrayCount; i++) {
                        if (i == rightIndex && isRightConnected) {
                            vr_src::TrackedDevicePose_t &pose = pGamePoseArray[i];
                            if (pose.bDeviceIsConnected && pose.bPoseIsValid && pose.eTrackingResult == vr_src::ETrackingResult::TrackingResult_Running_OK) {

                                // SteamVR
                                // +y is up
                                // +x is to the right
                                // -z is forward

                                // Skyrim
                                // +z is up
                                // +x is to the right
                                // +y is forward

                                // So, SteamVR -> Skyrim
                                // x <- x
                                // y <- -z
                                // z <- y

                                NiPoint3 openvrVelocity = { pose.vVelocity.v[0], pose.vVelocity.v[1], pose.vVelocity.v[2] };
                                NiPoint3 skyrimVelocity = { openvrVelocity.x, -openvrVelocity.z, openvrVelocity.y };
                                NiPoint3 velocityWorldspace = openvrToSkyrimWorldTransform * skyrimVelocity;
                                g_rightHand->controllerData.linearVelocities.pop_back();
                                g_rightHand->controllerData.linearVelocities.push_front(velocityWorldspace);

                                g_rightHand->controllerData.Recompute();

                                NiPoint3 openvrAngularVelocity = { pose.vAngularVelocity.v[0], pose.vAngularVelocity.v[1], pose.vAngularVelocity.v[2] };
                                NiPoint3 skyrimAngularVelocity = { openvrAngularVelocity.x, -openvrAngularVelocity.z, openvrAngularVelocity.y };
                                NiPoint3 angularVelocityWorldspace = openvrToSkyrimWorldTransform * skyrimAngularVelocity;
                                g_rightHand->controllerData.angularVelocities.pop_back();
                                g_rightHand->controllerData.angularVelocities.push_front(angularVelocityWorldspace);
                            }
                        }
                        else if (i == leftIndex && isLeftConnected) {
                            vr_src::TrackedDevicePose_t &pose = pGamePoseArray[i];
                            if (pose.bDeviceIsConnected && pose.bPoseIsValid && pose.eTrackingResult == vr_src::ETrackingResult::TrackingResult_Running_OK) {
                                NiPoint3 openvrVelocity = { pose.vVelocity.v[0], pose.vVelocity.v[1], pose.vVelocity.v[2] };
                                NiPoint3 skyrimVelocity = { openvrVelocity.x, -openvrVelocity.z, openvrVelocity.y };
                                NiPoint3 velocityWorldspace = openvrToSkyrimWorldTransform * skyrimVelocity;
                                g_leftHand->controllerData.linearVelocities.pop_back();
                                g_leftHand->controllerData.linearVelocities.push_front(velocityWorldspace);

                                g_leftHand->controllerData.Recompute();

                                NiPoint3 openvrAngularVelocity = { pose.vAngularVelocity.v[0], pose.vAngularVelocity.v[1], pose.vAngularVelocity.v[2] };
                                NiPoint3 skyrimAngularVelocity = { openvrAngularVelocity.x, -openvrAngularVelocity.z, openvrAngularVelocity.y };
                                NiPoint3 angularVelocityWorldspace = openvrToSkyrimWorldTransform * skyrimAngularVelocity;
                                g_leftHand->controllerData.angularVelocities.pop_back();
                                g_leftHand->controllerData.angularVelocities.push_front(angularVelocityWorldspace);
                            }
                        }
                    }
                }
            }
        }
    }
}


void UpdateShadowDelay()
{
    if (g_numShadowUpdates > 0) {
        if (g_shadowUpdateFrame != *g_currentFrameCounter) {
            if (g_numShadowUpdates > 1) {
                *g_nextShadowUpdateFrameCount = *g_currentFrameCounter;
                *g_iShadowUpdateFrameDelay = 1;
            }
            else { // == 1
                // Done
                *g_iShadowUpdateFrameDelay = g_savedShadowUpdateFrameDelay;
            }

            --g_numShadowUpdates;
        }
    }
}


float g_savedSpeedReduction = 0.f;
double g_lastHeldTime = 0.0;

std::unordered_set<hkpRigidBody *> g_thisFrameObjectMassRegisteredBodies{};
float g_totalMassThisFrameAccumulator = 0.f; // Accumulates for the curreent frame, can be in an intermediate state
float g_totalMassThisFrame = 0.f; // Always the final value

void RegisterObjectMass(hkpRigidBody *body, std::optional<float> massOverride)
{
    if (g_thisFrameObjectMassRegisteredBodies.contains(body)) return;

    g_thisFrameObjectMassRegisteredBodies.insert(body);

    if (massOverride) {
        g_totalMassThisFrameAccumulator += *massOverride;
    }
    else {
        float invMass = body->getMassInv();
        if (invMass != 0.f) {
            g_totalMassThisFrameAccumulator += 1.f / invMass;
        }
    }
}

void UpdateSpeedReduction()
{
    float speedReduction = 0.f;
    if (g_totalMassThisFrame > 0.f) {
        speedReduction = min(Config::options.slowMovementMaxReduction, powf(g_totalMassThisFrame, Config::options.slowMovementMassExponent) * Config::options.slowMovementMassProportion);
        g_lastHeldTime = g_currentFrameTime;
    }
    else if (g_currentFrameTime - g_lastHeldTime < Config::options.slowMovementFadeOutTime) {
        speedReduction = g_savedSpeedReduction * (1.f - (g_currentFrameTime - g_lastHeldTime) / Config::options.slowMovementFadeOutTime);
    }

    if (speedReduction != g_savedSpeedReduction) {
        // First just restore whatever our speed was before
        ModSpeedMult(*g_thePlayer, g_savedSpeedReduction);

        // Now modify the speed based on what we have held
        ModSpeedMult(*g_thePlayer, -speedReduction);

        g_savedSpeedReduction = speedReduction;
    }
}


struct DampedBodyData
{
    hkHalf linearDamping;
    hkHalf angularDamping;
    double endTime;
};
std::map<NiPointer<bhkRigidBody>, DampedBodyData> g_dampedBodyData{};

void AddDampedBody(bhkRigidBody *body)
{
    auto it = g_dampedBodyData.find(body);
    if (it == g_dampedBodyData.end()) {
        // Not already damped (e.g. by the other hand grabbing something)

        hkpMotion *motion = body->hkBody->getMotion();

        g_dampedBodyData[body] = { motion->m_motionState.m_linearDamping, motion->m_motionState.m_angularDamping, g_currentFrameTime + Config::options.grabFreezeNearbyVelocityTime };

        motion->m_motionState.m_linearDamping = hkHalf(Config::options.nearbyGrabLinearDamping);
        motion->m_motionState.m_angularDamping = hkHalf(Config::options.nearbyGrabAngularDamping);
    }
    else {
        // Already damped, just extend the time
        it->second.endTime = g_currentFrameTime + Config::options.grabFreezeNearbyVelocityTime;
    }
}

void ClearStaleDampedBodies(bhkWorld *world)
{
    static std::vector<std::pair<NiPointer<bhkRigidBody>, DampedBodyData>> toRemove{};

    for (auto &it = g_dampedBodyData.begin(); it != g_dampedBodyData.end();) {
        if (g_currentFrameTime > it->second.endTime) {
            auto body = it->first;
            toRemove.push_back(*it);
            it = g_dampedBodyData.erase(it);
        }
        else {
            ++it;
        }
    }

    if (!toRemove.empty()) {
        BSWriteLocker lock(&world->worldLock);

        for (auto &it : toRemove) {
            auto[body, data] = it;
            hkpMotion *motion = body->hkBody->getMotion();
            motion->m_motionState.m_linearDamping = data.linearDamping;
            motion->m_motionState.m_angularDamping = data.angularDamping;
        }

        toRemove.clear();
    }
}


std::set<NiPointer<bhkRigidBody>> g_playerSpaceBodies{};
std::unordered_set<bhkRigidBody *> g_playerSpaceBodiesShouldNotWarp{};

NiTransform g_prevNextRoomTransform{};
NiTransform g_prevRoomTransform{};

void RegisterPlayerSpaceBody(bhkRigidBody *body, bool allowWarp)
{
    g_playerSpaceBodies.insert(body);

    if (!allowWarp) {
        g_playerSpaceBodiesShouldNotWarp.insert(body);
    }
}

NiPoint3 g_prevDeltaVelocity{};
NiPoint3 g_prevDeltaVelocityWithVrikSmoothingOnly{};


void PrePhysicsStep(bhkWorld *world)
{
    int x = 0;
}

void PostPhysicsStep(bhkWorld *world)
{
    int x = 0;
}

bool g_prevDoWarp = false;
bool g_prevVelocityAdded = false;

float g_prevVrikOffset = 0.f;
float g_prevVrikSmoothingOffset = 0.f;

void SimulatePlayerSpace(bhkWorld *world)
{
    //_MESSAGE("%d SimulatePlayerSpace", *g_currentFrameCounter);

    // At the time this is called (in ApplyMovementDelta), the charcontroller has gotten a new position, but the player character's position won't be updated until the end of the frame (after rendering).

    PlayerCharacter *player = *g_thePlayer;

    NiAVObject *roomNode = player->unk3F0[PlayerCharacter::Node::kNode_RoomNode];
    if (!roomNode) return;

    NiAVObject *followNode = player->unk3F0[PlayerCharacter::Node::kNode_FollowNode];
    if (!followNode) return;

    NiPointer<bhkCharProxyController> controller = GetCharProxyController(*g_thePlayer);
    if (!controller) return;

    NiPoint3 playerVelocity = g_rightHand->avgPlayerVelocityWorldspace * *g_havokWorldScale;

    hkVector4 hkControllerPos;  controller->GetPositionImpl(hkControllerPos, true);
    NiPoint3 newControllerPos = HkVectorToNiPoint(hkControllerPos) * *g_inverseHavokWorldScale;

    NiTransform nextRoomTransform = roomNode->m_worldTransform;
    NiPoint3 predictedDelta = newControllerPos - followNode->m_worldTransform.pos;
    nextRoomTransform.pos += predictedDelta;

    NiPoint3 delta = (nextRoomTransform.pos - g_prevNextRoomTransform.pos) * *g_havokWorldScale;
    NiPoint3 deltaVelocity = delta / *g_deltaTime;

    float vrikZoffset = (Config::options.handleVrikOffsetting && g_vrikInterface) ? g_vrikInterface->getFinalCameraOffsettingAmount().z : 0.f;
    float vrikSmoothingZoffset = (Config::options.handleVrikOffsetting && g_vrikInterface) ? g_vrikInterface->getFinalSmoothingOffsettingAmount().z : 0.f;

    {
        NiPoint3 deltaVelocityWithSmoothing = deltaVelocity;
        deltaVelocityWithSmoothing.z += (vrikSmoothingZoffset - g_prevVrikSmoothingOffset) * *g_havokWorldScale / *g_deltaTime;
        g_prevDeltaVelocityWithVrikSmoothingOnly = deltaVelocityWithSmoothing;
    }

    delta.z += vrikZoffset * *g_havokWorldScale;
    deltaVelocity.z += (vrikZoffset - g_prevVrikOffset) * *g_havokWorldScale / *g_deltaTime;

    {
        BSWriteLocker lock(&world->worldLock);

        // There is potential for a body to be removed from the world between frames, or between being added to g_playerSpaceBodies and this call.
        std::erase_if(g_playerSpaceBodies, [](const NiPointer<bhkRigidBody> &body) {
            return !body->hkBody->isAddedToWorld();
        });

        NiTransform currentRoomTransform = roomNode->m_worldTransform;
        currentRoomTransform.pos *= *g_havokWorldScale;

        NiTransform prevRoomTransform = g_prevRoomTransform;
        prevRoomTransform.pos *= *g_havokWorldScale;

        NiTransform deltaRoomTransform = currentRoomTransform * InverseTransform(prevRoomTransform);
        auto [axis, angle] = QuaternionToAxisAngle(MatrixToQuaternion(deltaRoomTransform.rot));
        bool doWarp = angle > Config::options.playerSpaceMinDeltaAngleToWarp;

        bool actuallyDoWarp = doWarp || g_prevDoWarp;

        if (g_prevVelocityAdded) {
            for (bhkRigidBody *body : g_playerSpaceBodies) {
                // first subtract the previous velocity
                if (IsMoveableEntity(body->hkBody)) {
                    // Keyframed rigidBodies (like the hands) get their velocity zeroed when they step, so don't subtract anything
                    body->hkBody->m_motion.m_linearVelocity = NiPointToHkVector(HkVectorToNiPoint(body->hkBody->getLinearVelocity()) - g_prevDeltaVelocity);
                }
            }
        }

        if (actuallyDoWarp) {
            {
                static std::vector<hkpEntity *> recollideBodies;
                recollideBodies.clear();

                NiTransform prevRoomT = g_prevNextRoomTransform;
                prevRoomT.pos *= *g_havokWorldScale;

                NiTransform currentRoomT = nextRoomTransform;
                currentRoomT.pos *= *g_havokWorldScale;

                for (bhkRigidBody *body : g_playerSpaceBodies) {
                    if (g_playerSpaceBodiesShouldNotWarp.contains(body)) {
                        continue;
                    }

                    NiTransform currentTransform{};
                    currentTransform.pos = HkVectorToNiPoint(body->hkBody->getPosition());
                    currentTransform.rot = QuaternionToMatrix(HkQuatToNiQuat(body->hkBody->getRotation()));

                    NiTransform currentRoomSpace = InverseTransform(prevRoomT) * currentTransform;

                    NiTransform newTransform = currentRoomT * currentRoomSpace;
                    newTransform.pos.z += (vrikZoffset - g_prevVrikOffset) * *g_havokWorldScale;

                    NiPoint3 deltaPos = newTransform.pos - currentTransform.pos;

                    if (VectorLength(deltaPos) > 0.001f) {
                        bhkRigidBody_setActivated(body, true);
                        bhkEntity_setPositionAndRotation(body, NiPointToHkVector(newTransform.pos), NiQuatToHkQuat(MatrixToQuaternion(newTransform.rot))); // do NOT use the vfunc here, because the vfunc would apply bhkRigidBodyT transformations
                    }

                    recollideBodies.push_back(body->hkBody);
                }

                hkpWorld_reintegrateAndRecollideEntities(world->world, recollideBodies.data(), recollideBodies.size(), hkpWorld::ReintegrationRecollideMode::RR_MODE_RECOLLIDE_NARROWPHASE);
            }

            NiPoint3 predictedDeltaWithVrikOffset = predictedDelta;
            predictedDeltaWithVrikOffset.z += vrikZoffset;

            g_rightHand->MoveHandAndWeaponCollision(predictedDeltaWithVrikOffset * *g_havokWorldScale);
            g_leftHand->MoveHandAndWeaponCollision(predictedDeltaWithVrikOffset * *g_havokWorldScale);

            g_prevVelocityAdded = false;
        }
        else {
            for (bhkRigidBody *body : g_playerSpaceBodies) {
                // add the new velocity
                // Note: We set the velocity of the hands here too, but it doesn't matter since we're overwriting them right after this
                body->hkBody->m_motion.m_linearVelocity = NiPointToHkVector(HkVectorToNiPoint(body->hkBody->getLinearVelocity()) + deltaVelocity);
            }

            g_rightHand->MoveHandAndWeaponCollision(delta);
            g_leftHand->MoveHandAndWeaponCollision(delta);

            g_prevVelocityAdded = true;
        }

        g_playerSpaceBodies.clear();
        g_playerSpaceBodiesShouldNotWarp.clear();

        g_prevDoWarp = doWarp;
    }

    g_prevVrikOffset = vrikZoffset;
    g_prevVrikSmoothingOffset = vrikSmoothingZoffset;

    g_prevDeltaVelocity = deltaVelocity;
    g_prevNextRoomTransform = nextRoomTransform;

    g_prevRoomTransform = roomNode->m_worldTransform;
}


void Update()
{
    UpdateShadowDelay();

    if (!initComplete) return;

    PlayerCharacter *player = *g_thePlayer;
    if (!player || !player->GetNiNode()) return;

    g_currentFrameTime = GetTime();

    TESObjectCELL *cell = player->parentCell;
    if (!cell) return;

    NiPointer<bhkWorld> world = GetHavokWorldFromCell(cell);
    if (!world) {
        _MESSAGE("Could not get havok world from player cell");
        return;
    }

    bool reloadConfig = Config::options.reloadConfigIfModified;
#ifdef _DEBUG
    reloadConfig = true;
#endif // _DEBUG

    if (reloadConfig) {
        Config::ReloadIfModified();
    }

    if (world != g_physicsListener.world) {
        if (NiPointer<bhkWorld> oldWorld = g_physicsListener.world) {
            // If exists, remove the listener from the previous world
            _MESSAGE("Removing listeners and collision from old havok world");

            {
                BSWriteLocker lock(&oldWorld->worldLock);

                hkpWorld_removeContactListener(oldWorld->world, &g_physicsListener);
                hkpWorld_removeWorldPostSimulationListener(oldWorld->world, &g_physicsListener);

                if (Config::options.enableShadowUpdateFix) {
                    hkpWorld_removeIslandActivationListener(oldWorld->world, &g_activationListener);
                }

                g_rightHand->RemoveHandCollision(oldWorld);
                g_leftHand->RemoveHandCollision(oldWorld);

                g_rightHand->RemoveWeaponCollision(oldWorld);
                g_leftHand->RemoveWeaponCollision(oldWorld);

                g_physicsListener = PhysicsListener{};
            }
        }

        _MESSAGE("Adding listeners and collision to new havok world");

        {
            BSWriteLocker lock(&world->worldLock);

            AddHiggsCollisionLayer(world);

            hkpWorld_addContactListener(world->world, &g_physicsListener);
            hkpWorld_addWorldPostSimulationListener(world->world, &g_physicsListener);

            if (Config::options.enableShadowUpdateFix) {
                hkpWorld_addIslandActivationListener(world->world, &g_activationListener);
            }

            g_rightHand->CreateHandCollision(world);
            g_leftHand->CreateHandCollision(world);

            g_rightHand->CreateWeaponCollision(world);
            g_leftHand->CreateWeaponCollision(world);
        }

        g_physicsListener.world = world;
    }

    EnsureHiggsCollisionLayer(world);

    bool isRightHeld = g_rightHand->HasHeldKeyframed();
    bool isLeftHeld = g_leftHand->HasHeldKeyframed();

    Hand *firstHandToUpdate = g_rightHand;
    Hand *lastHandToUpdate = g_leftHand;
    if (isRightHeld && isLeftHeld && g_rightHand->selectedObject.handle == g_leftHand->selectedObject.handle) {
        // Both hands are holding something using the transform method, and they belong to the same object reference.
        // We need to see if one of the held nodes is a child of the other, and make sure to do the update for the child node last.

        NiPointer<TESObjectREFR> selectedObj;
        if (LookupREFRByHandle(g_rightHand->selectedObject.handle, selectedObj) && selectedObj->GetNiNode()) {
            NiPointer<NiAVObject> leftNode = GetNodeFromCollidable(g_leftHand->selectedObject.collidable);
            NiPointer<NiAVObject> rightNode = GetNodeFromCollidable(g_rightHand->selectedObject.collidable);
            if (leftNode && rightNode) {
                if (DoesNodeHaveNode(leftNode, rightNode)) {
                    // Right is the child
                    firstHandToUpdate = g_leftHand;
                    lastHandToUpdate = g_rightHand;
                }
                else if (DoesNodeHaveNode(rightNode, leftNode)) {
                    // Left is the child
                    firstHandToUpdate = g_rightHand;
                    lastHandToUpdate = g_leftHand;
                }
            }
        }
    }

    g_totalMassThisFrameAccumulator = 0.f; // reset this before hand updates which add to it
    g_thisFrameObjectMassRegisteredBodies.clear();

    firstHandToUpdate->PreUpdate(*lastHandToUpdate, world);
    lastHandToUpdate->PreUpdate(*firstHandToUpdate, world);

    firstHandToUpdate->Update(*lastHandToUpdate, world);
    lastHandToUpdate->Update(*firstHandToUpdate, world);

    firstHandToUpdate->PostUpdate(*lastHandToUpdate, world);
    lastHandToUpdate->PostUpdate(*firstHandToUpdate, world);

    ClearStaleDampedBodies(world);

    g_totalMassThisFrame = g_totalMassThisFrameAccumulator; // commit the total mass for this frame

    if (Config::options.slowMovementWhenObjectIsHeld) {
        UpdateSpeedReduction();
    }
}

void DebugDrawSphere(const NiTransform &transform, const NiColorA &color)
{
#ifdef _DEBUG
    NiTransform axisX = transform;
    axisX.pos += RightVector(axisX.rot) * axisX.scale;
    axisX.scale = 0.2f;
    NiTransform axisY = transform;
    axisY.pos += ForwardVector(axisX.rot) * axisY.scale;
    axisY.scale = 0.2f;
    NiTransform axisZ = transform;
    axisZ.pos += UpVector(axisX.rot) * axisZ.scale;
    axisZ.scale = 0.2f;

    NiTransform sphere = transform;
    sphere.scale -= 0.2f;
    Draw::DrawSphere(sphere, color);
    Draw::DrawSphere(axisX, { 1.f, 0.f, 0.f, 1.f });
    Draw::DrawSphere(axisY, { 0.f, 1.f, 0.f, 1.f });
    Draw::DrawSphere(axisZ, { 0.f, 0.f, 1.f, 1.f });
#endif // _DEBUG
}

std::unordered_map<std::string_view, DebugTransform> g_debugDrawTransforms{};

void DebugDraw()
{
#ifdef _DEBUG
    for (auto &pair : g_debugDrawTransforms) {
        DebugDrawSphere(pair.second.transform, pair.second.color);
    }
#endif // _DEBUG
}

void RegisterDebugTransform(const std::string_view &name, const DebugTransform &transform)
{
#ifdef _DEBUG
    g_debugDrawTransforms[name] = transform;
#endif // _DEBUG
}

void UnregisterDebugTransform(const std::string_view &name)
{
#ifdef _DEBUG
    g_debugDrawTransforms.erase(name);
#endif // _DEBUG
}


bool WaitPosesCB(vr_src::TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, vr_src::TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount)
{
    PlayerCharacter *player = *g_thePlayer;
    if (!player || !player->GetNiNode()) return true;

    NiPointer<NiAVObject> hmdNode = player->unk3F0[PlayerCharacter::Node::kNode_HmdNode];
    if (!hmdNode) return true;

    FillControllerVelocities(hmdNode, pGamePoseArray, unGamePoseArrayCount);

    return true;
}


void ControllerStateCB(uint32_t unControllerDeviceIndex, vr_src::VRControllerState001_t *pControllerState, uint32_t unControllerStateSize, bool& state)
{
    if (!initComplete) return;

    PlayerCharacter *player = *g_thePlayer;
    if (!player || !player->GetNiNode())
        return;

    vr_src::ETrackedControllerRole rightControllerRole = vr_src::ETrackedControllerRole::TrackedControllerRole_RightHand;
    vr_src::TrackedDeviceIndex_t rightController = (*g_openVR)->vrSystem->GetTrackedDeviceIndexForControllerRole(rightControllerRole);

    vr_src::ETrackedControllerRole leftControllerRole = vr_src::ETrackedControllerRole::TrackedControllerRole_LeftHand;
    vr_src::TrackedDeviceIndex_t leftController = (*g_openVR)->vrSystem->GetTrackedDeviceIndexForControllerRole(leftControllerRole);

    if (unControllerDeviceIndex == rightController) {
        g_rightHand->ControllerStateUpdate(unControllerDeviceIndex, pControllerState);
    }
    else if (unControllerDeviceIndex == leftController) {
        g_leftHand->ControllerStateUpdate(unControllerDeviceIndex, pControllerState);
    }
}


void ShowErrorBox(const char *errorString)
{
    int msgboxID = MessageBox(
        NULL,
        (LPCTSTR)errorString,
        (LPCTSTR)"HIGGS Fatal Error",
        MB_ICONERROR | MB_OK | MB_TASKMODAL
    );
}

void ShowErrorBoxAndLog(const char *errorString)
{
    _ERROR(errorString);
    ShowErrorBox(errorString);
}

void ShowErrorBoxAndTerminate(const char *errorString)
{
    ShowErrorBoxAndLog(errorString);
    *((int *)0) = 0xDEADBEEF; // crash
}


class HitEventHandler : public BSTEventSink <TESHitEvent>
{
public:
    virtual	EventResult ReceiveEvent(TESHitEvent *evn, EventDispatcher<TESHitEvent> *dispatcher)
    {
        PlayerCharacter *player = *g_thePlayer;
        if (evn->caster == player) {
            bool isLeftHanded = *g_leftHandedMode;
            TESForm *rightWeapon = player->GetEquippedObject(isLeftHanded);
            TESForm *leftWeapon = player->GetEquippedObject(!isLeftHanded);

            TESForm *weapon = LookupFormByID(evn->sourceFormID);
            if (weapon == rightWeapon) {
                g_rightHand->weaponHitTime = g_currentFrameTime;
            }
            else if (weapon == leftWeapon) {
                g_leftHand->weaponHitTime = g_currentFrameTime;
            }
        }

        return kEvent_Continue;
    }
};
HitEventHandler hitEventHandler;

std::unordered_map<UInt32, int> g_ignoredCollisionGroups{};
std::mutex g_ignoredCollisionGroupsLock{};


void LateMainThreadUpdate()
{
    //_MESSAGE("%d LateMainThreadUpdate", *g_currentFrameCounter);

    g_rightHand->LateMainThreadUpdate();
    g_leftHand->LateMainThreadUpdate();
}


typedef bool(*_PlayerCharacter_UpdateVRFollow)(PlayerCharacter *_this, float a_deltaWorldTime, float a_deltaGameTime);
RelocAddr<_PlayerCharacter_UpdateVRFollow> PlayerCharacter_UpdateVRFollow(0x6AD080);

void PlayerPostApplyMovementDeltaUpdate()
{
    //_MESSAGE("%d PlayerPostApplyMovementDeltaUpdate", *g_currentFrameCounter);

    // Clear out old ignored collision groups.
    // We do this here because this is the same thread that will be doing the character controller integrate() calls that will be using the ignored collision groups.
    if (g_ignoredCollisionGroups.size() > 0) { // quick check without locking first
        std::scoped_lock lock(g_ignoredCollisionGroupsLock);

        for (auto it = g_ignoredCollisionGroups.begin(); it != g_ignoredCollisionGroups.end();) {
            if (*g_currentFrameCounter - it->second > 5) {
                it = g_ignoredCollisionGroups.erase(it);
            }
            else {
                ++it;
            }
        }
    }

    PlayerCharacter *player = *g_thePlayer;
    if (TESObjectCELL *cell = player->parentCell) {
        if (NiPointer<bhkWorld> world = GetHavokWorldFromCell(cell)) {
            SimulatePlayerSpace(world);
        }
    }
}


void ProcessPlayerProxyCastCollector(hkpAllCdPointCollector *collector)
{
    UInt32 playerCollisionGroup = g_rightHand->playerCollisionGroup;

    for (int i = 0; i < collector->getNumHits(); i++) {
        hkpRootCdPoint &hit = collector->getHits()[i];
        UInt32 collisionGroupA = hit.m_rootCollidableA->getCollisionFilterInfo() >> 16;
        UInt32 collisionGroupB = hit.m_rootCollidableB->getCollisionFilterInfo() >> 16;

        UInt32 otherGroup = collisionGroupA == playerCollisionGroup ? collisionGroupB : collisionGroupA;

        if (g_rightHand->ShouldIgnoreCollisionGroup(otherGroup) || g_leftHand->ShouldIgnoreCollisionGroup(otherGroup)) {
            {
                std::scoped_lock lock(g_ignoredCollisionGroupsLock);
                g_ignoredCollisionGroups[otherGroup] = *g_currentFrameCounter;
            }

            // remove the hit by moving the last hit into its place
            collector->getHits()[i] = collector->getHits()[collector->getNumHits() - 1];
            collector->getHits().m_size -= 1;
            i -= 1;
        }
        else {
            {
                std::scoped_lock lock(g_ignoredCollisionGroupsLock);

                if (auto it = g_ignoredCollisionGroups.find(otherGroup); it != g_ignoredCollisionGroups.end() && *g_currentFrameCounter - it->second <= 1) {
                    // We were ignoring this group last frame, so continue ignoring it. 
                    // This way we stop ignoring it only if it stops coming up in the linear cast query that's done every frame for the player character proxy.
                    it->second = *g_currentFrameCounter;

                    collector->getHits()[i] = collector->getHits()[collector->getNumHits() - 1];
                    collector->getHits().m_size -= 1;
                    i -= 1;

                    continue;
                }
            }

            // Everything else

            const hkpCollidable *otherCollidable = collisionGroupA == playerCollisionGroup ? hit.m_rootCollidableB : hit.m_rootCollidableA;
            if (hkpRigidBody *rigidBody = hkpGetRigidBody(otherCollidable)) {
                UInt32 otherLayer = otherCollidable->getCollisionFilterInfo() & 0x7f;
                if (otherLayer == BGSCollisionLayer::kCollisionLayer_Clutter || otherLayer == BGSCollisionLayer::kCollisionLayer_Weapon) {
                    float massInv = rigidBody->getMassInv();
                    float mass = massInv != 0 ? 1.f / massInv : (std::numeric_limits<float>::max)();

                    // TODO: We need stuff above the mass threshold to not collide with the player if it's a "contained" object
                    //       - This is somewhat mitigated already by getting rid of most clutter collision
                    if (mass < Config::options.minCollideClutterMass) {
                        collector->getHits()[i] = collector->getHits()[collector->getNumHits() - 1];
                        collector->getHits().m_size -= 1;
                        i -= 1;
                    }
                    else {
                        hkVector4 pointVelocity; rigidBody->getPointVelocity(hit.m_contact.getPosition(), pointVelocity);
                        hkVector4 normal = hit.m_contact.getNormal();
                        float speedInNormalDirection = pointVelocity.dot3(normal);

                        if (speedInNormalDirection > Config::options.dontCollideClutterMinVelocity) {
                            { // Ignore the entire object after this
                                std::scoped_lock lock(g_ignoredCollisionGroupsLock);
                                g_ignoredCollisionGroups[otherGroup] = *g_currentFrameCounter;
                            }

                            collector->getHits()[i] = collector->getHits()[collector->getNumHits() - 1];
                            collector->getHits().m_size -= 1;
                            i -= 1;
                        }
                    }
                }
            }
        }
    }
}


extern "C" {
    void OnDataLoaded()
    {
        const ModInfo *modInfo = DataHandler::GetSingleton()->LookupModByName("higgs_vr.esp");
        if (!modInfo) {
            ShowErrorBoxAndTerminate("[CRITICAL] Could not get modinfo. Most likely the higgs esp doesn't exist.");
            return;
        }

        if (!modInfo->IsActive()) {
            ShowErrorBoxAndTerminate("[CRITICAL] The higgs esp exists, but is not active. Make sure the esp is enabled in your mod manager.");
            return;
        }

        TESForm *shaderForm = LookupFormByID(GetFullFormID(modInfo, 0x6F00));
        if (!shaderForm) {
            ShowErrorBoxAndTerminate("Failed to get slected item shader form");
            return;
        }
        g_itemSelectedShader = DYNAMIC_CAST(shaderForm, TESForm, TESEffectShader);
        if (!g_itemSelectedShader) {
            ShowErrorBoxAndTerminate("Failed to cast selected item shader form");
            return;
        }
        
        shaderForm = LookupFormByID(GetFullFormID(modInfo, 0x6F01));
        if (!shaderForm) {
            ShowErrorBoxAndTerminate("Failed to get slected item off limits shader form");
            return;
        }
        g_itemSelectedShaderOffLimits = DYNAMIC_CAST(shaderForm, TESForm, TESEffectShader);
        if (!g_itemSelectedShaderOffLimits) {
            ShowErrorBoxAndTerminate("Failed to cast selected item off limits shader form");
            return;
        }
        
        MenuManager * menuManager = MenuManager::GetSingleton();
        if (menuManager) {
            menuManager->MenuOpenCloseEventDispatcher()->AddEventSink(&MenuChecker::menuEvent);
        }

        EventDispatcherList *eventDispatcherList = GetEventDispatcherList();
        if (eventDispatcherList) {
            ((EventDispatcher<TESHitEvent> *)(&eventDispatcherList->unk630))->AddEventSink(&hitEventHandler);
        }
        else {
            ShowErrorBoxAndTerminate("Failed to get event dispatcher list");
            return;
        }

        g_isVrikPresent = GetModuleHandle("vrik") != NULL;

        // Need to heap-allocate and "leak" anything with NiPointers since if they're statically allocated we crash when the game exits and these objects destruct

        // Init both hands
        NiPoint3 rightPalm = Config::options.palmPosition;
        NiPoint3 leftPalm = rightPalm;
        leftPalm.x *= -1;

        BSFixedString rightFingerNames[5][3] = {
            {
                BSFixedString("NPC R Finger00 [RF00]"),
                BSFixedString("NPC R Finger01 [RF01]"),
                BSFixedString("NPC R Finger02 [RF02]")
            },
            {
                BSFixedString("NPC R Finger10 [RF10]"),
                BSFixedString("NPC R Finger11 [RF11]"),
                BSFixedString("NPC R Finger12 [RF12]")
            },
            {
                BSFixedString("NPC R Finger20 [RF20]"),
                BSFixedString("NPC R Finger21 [RF21]"),
                BSFixedString("NPC R Finger22 [RF22]")
            },
            {
                BSFixedString("NPC R Finger30 [RF30]"),
                BSFixedString("NPC R Finger31 [RF31]"),
                BSFixedString("NPC R Finger32 [RF32]")
            },
            {
                BSFixedString("NPC R Finger40 [RF40]"),
                BSFixedString("NPC R Finger41 [RF41]"),
                BSFixedString("NPC R Finger42 [RF42]")
            },
        };

        BSFixedString leftFingerNames[5][3] = {
            {
                BSFixedString("NPC L Finger00 [LF00]"),
                BSFixedString("NPC L Finger01 [LF01]"),
                BSFixedString("NPC L Finger02 [LF02]")
            },
            {
                BSFixedString("NPC L Finger10 [LF10]"),
                BSFixedString("NPC L Finger11 [LF11]"),
                BSFixedString("NPC L Finger12 [LF12]")
            },
            {
                BSFixedString("NPC L Finger20 [LF20]"),
                BSFixedString("NPC L Finger21 [LF21]"),
                BSFixedString("NPC L Finger22 [LF22]")
            },
            {
                BSFixedString("NPC L Finger30 [LF30]"),
                BSFixedString("NPC L Finger31 [LF31]"),
                BSFixedString("NPC L Finger32 [LF32]")
            },
            {
                BSFixedString("NPC L Finger40 [LF40]"),
                BSFixedString("NPC L Finger41 [LF41]"),
                BSFixedString("NPC L Finger42 [LF42]")
            },
        };

        g_rightHand = new Hand(false, "R", "NPC R Hand [RHnd]", "RightWandNode", "HIGGS:GrabR", rightFingerNames, rightPalm, Config::options.rolloverOffsetRight, Config::options.delayRightGripInput);
        g_leftHand = new Hand(true, "L", "NPC L Hand [LHnd]", "LeftWandNode", "HIGGS:GrabL", leftFingerNames, leftPalm, Config::options.rolloverOffsetLeft, Config::options.delayLeftGripInput);

        if (!g_rightHand || !g_leftHand) {
            ShowErrorBoxAndTerminate("[CRITICAL] Couldn't allocate memory");
            return;
        }

        g_rightHand->itemSelectedShader = g_itemSelectedShader;
        g_rightHand->itemSelectedShaderOffLimits = g_itemSelectedShaderOffLimits;

        g_leftHand->itemSelectedShader = g_itemSelectedShader;
        g_leftHand->itemSelectedShaderOffLimits = g_itemSelectedShaderOffLimits;

        NiMatrix33 rightRolloverRotation = EulerToMatrix(Config::options.rolloverRotation);
        NiMatrix33 leftRolloverRotation = EulerToMatrix({ Config::options.rolloverRotation.x, -Config::options.rolloverRotation.y, -Config::options.rolloverRotation.z });

        g_rightHand->rolloverRotation = rightRolloverRotation;
        g_rightHand->rolloverScale = Config::options.rolloverScale;

        g_leftHand->rolloverRotation = leftRolloverRotation;
        g_leftHand->rolloverScale = Config::options.rolloverScale;

        g_controllerType = (*g_openVR)->GetControllerType();
        _MESSAGE("Controller type detected as %d", g_controllerType);

        InputManager *inputManager = InputManager::GetSingleton();
        InputStringHolder *inputStringHolder = InputStringHolder::GetSingleton();
        if (inputManager && inputStringHolder) {
            UInt32 activateKey = inputManager->GetMappedKey(inputStringHolder->activate, kDeviceType_OculusPrimary, InputManager::kContext_Gameplay);
            _MESSAGE("Activate key detected as %d", activateKey);
            if (activateKey == vr_src::EVRButtonId::k_EButton_Grip) {
                _MESSAGE("Activate key is assigned to grip. The activate icon will be replaced with a grip icon");
                g_isActivateBoundToGrip = true;
            }
        }

        if (Config::options.disableRolloverRumble) {
            _MESSAGE("Disabling rollover rumble");
            Setting	* setting = GetINISetting("fActivateRumbleIntensity:VRInput");
            setting->SetDouble(0);
        }

        if (Config::options.alwaysShowHands) {
            _MESSAGE("Setting bAlwaysShowHands to true");
            Setting	*setting = GetINISetting("bAlwaysShowHands:VR");
            setting->SetDouble(1);
        }

        if (Config::options.enableHavokFix) {
            // Overwrite these as the user may have set them to to something of their own choosing in the game's ini file
            *g_uMaxNumPhysicsStepsPerUpdate = Config::options.maxNumPhysicsStepsPerUpdate;
            *g_uMaxNumPhysicsStepsPerUpdateComplex = Config::options.maxNumPhysicsStepsPerUpdateComplex;
        }

        if (Config::options.minCollideClutterMass != 0.f) {
            *g_fMoveLimitMass = Config::options.minCollideClutterMass;
        }

        initComplete = true;
        _MESSAGE("Successfully loaded all forms");
    }

    void OnInputLoaded()
    {

    }

    // Listener for SKSE Messages
    void OnSKSEMessage(SKSEMessagingInterface::Message* msg)
    {
        if (msg) {
            if (msg->type == SKSEMessagingInterface::kMessage_InputLoaded) {
                OnInputLoaded();
            }
            else if (msg->type == SKSEMessagingInterface::kMessage_DataLoaded) {
                OnDataLoaded();
            }
            else if (msg->type == SKSEMessagingInterface::kMessage_PostLoad) {
                // Register our own mod api listener
                g_messaging->RegisterListener(g_pluginHandle, nullptr, HiggsPluginAPI::ModMessageHandler);
            }
            else if (msg->type == SKSEMessagingInterface::kMessage_PostPostLoad) {
                // Get the VRIK plugin API
                g_vrikInterface = vrikPluginApi::getVrikInterface001(g_pluginHandle, g_messaging);
                if (g_vrikInterface) {
                    _MESSAGE("Successfully got VRIK api");

                    unsigned int vrikVersion = g_vrikInterface->getBuildNumber();
                    if (vrikVersion < 80400) {
                        ShowErrorBoxAndTerminate("[CRITICAL] You are using VRIK, but its version is lower than HIGGS supports. If you want to use VRIK alongside HIGGS, get the latest version of VRIK and try again.");
                    }
                }
                else {
                    _MESSAGE("Did not get VRIK api. This is okay.");
                }
            }
        }
    }

    bool SKSEPlugin_Query(const SKSEInterface* skse, PluginInfo* info)
    {
        gLog.OpenRelative(CSIDL_MYDOCUMENTS, "\\My Games\\Skyrim VR\\SKSE\\higgs_vr.log");
        gLog.SetPrintLevel(IDebugLog::kLevel_Message);
        gLog.SetLogLevel(IDebugLog::kLevel_Message);

        _MESSAGE("HIGGS VR v%s", FPVR_VERSION_VERSTRING);

        info->infoVersion = PluginInfo::kInfoVersion;
        info->name = "HIGGS";
        info->version = FPVR_VERSION_MAJOR;

        g_pluginHandle = skse->GetPluginHandle();

        if (skse->isEditor) {
            _FATALERROR("[FATAL ERROR] Loaded in editor, marking as incompatible!\n");
            return false;
        }
        else if (skse->runtimeVersion != RUNTIME_VR_VERSION_1_4_15) {
            _FATALERROR("[FATAL ERROR] Unsupported runtime version %08X!\n", skse->runtimeVersion);
            return false;
        }

        return true;
    }

    bool SKSEPlugin_Load(const SKSEInterface * skse)
    {	// Called by SKSE to load this plugin
        _MESSAGE("HIGGS loaded");

        if (Config::ReadConfigOptions()) {
            _MESSAGE("Successfully read config parameters");
        }
        else {
            _WARNING("[WARNING] Failed to read config options. Using defaults instead.");
        }

        gLog.SetPrintLevel((IDebugLog::LogLevel)Config::options.logLevel);
        gLog.SetLogLevel((IDebugLog::LogLevel)Config::options.logLevel);

        _MESSAGE("Registering for SKSE messages");
        g_messaging = (SKSEMessagingInterface*)skse->QueryInterface(kInterface_Messaging);
        g_messaging->RegisterListener(g_pluginHandle, "SKSE", OnSKSEMessage);

        g_vrInterface = (SKSEVRInterface *)skse->QueryInterface(kInterface_VR);
        if (!g_vrInterface) {
            ShowErrorBoxAndLog("[CRITICAL] Couldn't get SKSE VR interface. You probably have an outdated SKSE version.");
            return false;
        }
        g_vrInterface->RegisterForControllerState(g_pluginHandle, 66, ControllerStateCB);
        g_vrInterface->RegisterForPoses(g_pluginHandle, 66, WaitPosesCB);

        g_papyrus = (SKSEPapyrusInterface *)skse->QueryInterface(kInterface_Papyrus);
        if (!g_papyrus) {
            ShowErrorBoxAndLog("[CRITICAL] Couldn't get Papyrus interface");
            return false;
        }
        if (g_papyrus->Register(PapyrusAPI::RegisterPapyrusFuncs)) {
            _MESSAGE("Successfully registered papyrus functions");
        }

        g_taskInterface = (SKSETaskInterface *)skse->QueryInterface(kInterface_Task);
        if (!g_taskInterface) {
            ShowErrorBoxAndLog("[CRITICAL] Could not get SKSE task interface");
            return false;
        }

        g_trampoline = (SKSETrampolineInterface *)skse->QueryInterface(kInterface_Trampoline);
        if (!g_trampoline) {
            _WARNING("Couldn't get trampoline interface");
        }
        if (!TryHook()) {
            ShowErrorBoxAndLog("[CRITICAL] Failed to perform hooks");
            return false;
        }

        g_timer.Start();

        return true;
    }
};
