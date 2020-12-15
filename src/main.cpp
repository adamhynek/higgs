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
#include "skse64/PluginAPI.h"  // SKSEInterface, PluginInfo
#include "skse64/GameRTTI.h"
#include "skse64/GameSettings.h"
#include "skse64/NiNodes.h"
#include "skse64/NiObjects.h"
#include "skse64/NiExtraData.h"
#include "skse64/GameData.h"
#include "skse64/GameForms.h"
#include "skse64/PapyrusActor.h"
#include "skse64/GameVR.h"
#include "skse64_common/SafeWrite.h"
#include "skse64_common/BranchTrampoline.h"

#include <ShlObj.h>  // CSIDL_MYDOCUMENTS

#include "grabber.h"
#include "version.h"
#include "utils.h"
#include "config.h"
#include "menu_checker.h"
#include "effects.h"
#include "offsets.h"
#include "hooks.h"
#include "vrikinterface001.h"


// SKSE globals
static PluginHandle	g_pluginHandle = kPluginHandle_Invalid;
static SKSEMessagingInterface *g_messaging = nullptr;

SKSEVRInterface *g_vrInterface = nullptr;
SKSETrampolineInterface *g_trampoline = nullptr;

vrikPluginApi::IVrikInterface001 * g_vrikInterface;

NiMatrix33 g_rolloverRotation; // Set on plugin load

BSFixedString rolloverNodeStr("WSActivateRollover");

bool g_isLoaded = false;

TESEffectShader *g_itemSelectedShader = nullptr;
TESEffectShader *g_itemSelectedShaderOffLimits = nullptr;

bool g_hasSavedRumbleIntensity = false;
float g_normalRumbleIntensity;
bool g_hasSavedRollover = false;
NiTransform g_normalRolloverTransform;

bool initComplete = false; // Whether grabbers have been initialized

Grabber *g_rightGrabber;
Grabber *g_leftGrabber;

std::unordered_map<ShaderReferenceEffect *, std::unordered_set<BSGeometry *>> *g_shaderNodes;


bool TryHook()
{
	// This should be sized to the actual amount used by your trampoline
	static const size_t TRAMPOLINE_SIZE = 4096;

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


bool WaitPosesCB(vr_src::TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, vr_src::TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount)
{
	if (!initComplete || !g_isLoaded) return true;

	if (MenuChecker::isGameStopped()) return true;

	PlayerCharacter *player = *g_thePlayer;
	if (!player || !player->loadedState || !player->loadedState->node)
		return true;

	NiAVObject *rootObj = GetHighestParent(player->loadedState->node);
	if (!rootObj)
		return true;

	g_currentFrameTime = GetTime();

	NiNode *rootNode = rootObj->GetAsNiNode();
	if (!rootNode)
		return true;

	static BSFixedString playerWorldNodeName("PlayerWorldNode");
	NiAVObject *playerWorldObj = rootNode->GetObjectByName(&playerWorldNodeName.data);
	if (!playerWorldObj)
		return true;

	NiNode *playerWorldNode = playerWorldObj->GetAsNiNode();
	if (!playerWorldNode)
		return true;

	static BSFixedString rightWandStr("RightWandNode");
	NiAVObject *rightWandObj = playerWorldObj->GetObjectByName(&rightWandStr.data);
	if (!rightWandObj) {
		return true;
	}
	NiNode *rightWandNode = rightWandObj->GetAsNiNode();

	static BSFixedString leftWandStr("LeftWandNode");
	NiAVObject *leftWandObj = playerWorldObj->GetObjectByName(&leftWandStr.data);
	if (!leftWandObj)
		return true;
	NiNode *leftWandNode = leftWandObj->GetAsNiNode();

	static BSFixedString hmdStr("HmdNode");
	NiAVObject *hmdObj = playerWorldObj->GetObjectByName(&hmdStr.data);
	if (!hmdObj)
		return true;
	NiNode *hmdNode = hmdObj->GetAsNiNode();

	if (g_openVR && *g_openVR) {
		BSOpenVR *openVR = *g_openVR;
		vr_src::IVRSystem *vrSystem = openVR->vrSystem;
		if (vrSystem) {
			vr_src::TrackedDeviceIndex_t rightIndex = vrSystem->GetTrackedDeviceIndexForControllerRole(vr_src::ETrackedControllerRole::TrackedControllerRole_RightHand);
			vr_src::TrackedDeviceIndex_t leftIndex = vrSystem->GetTrackedDeviceIndexForControllerRole(vr_src::ETrackedControllerRole::TrackedControllerRole_LeftHand);
			vr_src::TrackedDeviceIndex_t hmdIndex = vr_src::k_unTrackedDeviceIndex_Hmd;

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

								g_rightGrabber->controllerVelocities.pop_back();
								g_rightGrabber->controllerVelocities.push_front(velocityWorldspace);
							}
						}
						else if (i == leftIndex && isLeftConnected) {
							vr_src::TrackedDevicePose_t &pose = pGamePoseArray[i];
							if (pose.bDeviceIsConnected && pose.bPoseIsValid && pose.eTrackingResult == vr_src::ETrackingResult::TrackingResult_Running_OK) {
								NiPoint3 openvrVelocity = { pose.vVelocity.v[0], pose.vVelocity.v[1], pose.vVelocity.v[2] };
								NiPoint3 skyrimVelocity = { openvrVelocity.x, -openvrVelocity.z, openvrVelocity.y };

								NiPoint3 velocityWorldspace = openvrToSkyrimWorldTransform * skyrimVelocity;

								g_leftGrabber->controllerVelocities.pop_back();
								g_leftGrabber->controllerVelocities.push_front(velocityWorldspace);
							}
						}
					}
				}
			}
		}
	}

	std::pair<bool, bool> validItems = AreEquippedItemsValid(player);

	bool isLeftHanded = *g_leftHandedMode;

	bool isRightHeld = g_rightGrabber->state == Grabber::State::HeldInit || g_rightGrabber->state == Grabber::State::Held;
	bool isLeftHeld = g_leftGrabber->state == Grabber::State::HeldInit || g_leftGrabber->state == Grabber::State::Held;

	Grabber *firstGrabberToUpdate = g_rightGrabber;
	Grabber *lastGrabberToUpdate = g_leftGrabber;
	if (isRightHeld && isLeftHeld && g_rightGrabber->selectedObject.handle == g_leftGrabber->selectedObject.handle) {
		// Both hands are holding something using the transform method, and they belong to the same object reference.
		// We need to see if one of the held nodes is a child of the other, and make sure to do the update for the child node last.

		NiPointer<TESObjectREFR> selectedObj;
		if (LookupREFRByHandle(g_rightGrabber->selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {
			NiAVObject *leftNode = FindCollidableNode(g_leftGrabber->selectedObject.collidable);
			NiAVObject *rightNode = FindCollidableNode(g_rightGrabber->selectedObject.collidable);
			if (leftNode && rightNode) {
				if (DoesNodeHaveNode(leftNode, rightNode)) {
					// Right is the child
					firstGrabberToUpdate = g_leftGrabber;
					lastGrabberToUpdate = g_rightGrabber;
				}
				else if (DoesNodeHaveNode(rightNode, leftNode)) {
					// Left is the child
					firstGrabberToUpdate = g_rightGrabber;
					lastGrabberToUpdate = g_leftGrabber;
				}
			}
		}
	}

	firstGrabberToUpdate->PoseUpdate(*g_leftGrabber, isLeftHanded ? validItems.second : validItems.first, playerWorldNode);
	lastGrabberToUpdate->PoseUpdate(*g_rightGrabber, isLeftHanded ? validItems.first : validItems.second, playerWorldNode);

	if (g_rightGrabber->IsSafeToClearSavedCollision() && g_leftGrabber->IsSafeToClearSavedCollision()) {
		// cleanup the collision id map to prevent mem leaks when an item is destroyed (i.e. 'activated', etc.) while holding / pulling it
		CollisionInfo::ClearCollisionMap();
	}

	bool displayLeft = g_leftGrabber->ShouldDisplayRollover();
	bool displayRight = g_rightGrabber->ShouldDisplayRollover();

	if (displayRight || displayLeft) {
		// Something is grabbed

		Setting	* activateRumbleIntensitySetting = GetINISetting("fActivateRumbleIntensity:VRInput");
		if (!g_hasSavedRumbleIntensity) {
			g_hasSavedRumbleIntensity = true;
			g_normalRumbleIntensity = activateRumbleIntensitySetting->data.f32;
		}
		activateRumbleIntensitySetting->SetDouble(0);

		if (!g_hasSavedRollover) {
			NiAVObject *rolloverNode = playerWorldObj->GetObjectByName(&rolloverNodeStr.data);
			if (rolloverNode) {
				g_normalRolloverTransform = rolloverNode->m_localTransform;
				g_hasSavedRollover = true;
			}
		}

		if (displayRight && displayLeft) {
			// Pick whichever hand grabbed last
			if (g_leftGrabber->rolloverDisplayTime > g_rightGrabber->rolloverDisplayTime) {
				NiAVObject *rolloverNode = leftWandNode->GetObjectByName(&rolloverNodeStr.data);
				if (!rolloverNode) {
					// Switch menu to left hand if it's on the right
					rolloverNode = rightWandNode->GetObjectByName(&rolloverNodeStr.data);
					leftWandNode->AttachChild(rolloverNode, false);
					rightWandNode->RemoveChild(rolloverNode);
				}
				g_leftGrabber->SetupRollover(rolloverNode, isLeftHanded);
			}
			else {
				NiAVObject *rolloverNode = rightWandNode->GetObjectByName(&rolloverNodeStr.data);
				if (!rolloverNode) {
					// Switch menu to right hand if it's on the left
					rolloverNode = leftWandNode->GetObjectByName(&rolloverNodeStr.data);
					rightWandNode->AttachChild(rolloverNode, false);
					leftWandNode->RemoveChild(rolloverNode);
				}
				g_rightGrabber->SetupRollover(rolloverNode, isLeftHanded);
			}
		}
		else if (displayRight) {
			NiAVObject *rolloverNode = rightWandNode->GetObjectByName(&rolloverNodeStr.data);
			if (!rolloverNode) {
				// Switch menu to right hand if it's on the left
				rolloverNode = leftWandNode->GetObjectByName(&rolloverNodeStr.data);
				rightWandNode->AttachChild(rolloverNode, false);
				leftWandNode->RemoveChild(rolloverNode);
			}
			g_rightGrabber->SetupRollover(rolloverNode, isLeftHanded);
		}
		else if (displayLeft) {
			NiAVObject *rolloverNode = leftWandNode->GetObjectByName(&rolloverNodeStr.data);
			if (!rolloverNode) {
				// Switch menu to left hand if it's on the right
				rolloverNode = rightWandNode->GetObjectByName(&rolloverNodeStr.data);
				leftWandNode->AttachChild(rolloverNode, false);
				rightWandNode->RemoveChild(rolloverNode);
			}
			g_leftGrabber->SetupRollover(rolloverNode, isLeftHanded);
		}
	}
	else {
		// Nothing is grabbed

		NiNode *mainWandNode = isLeftHanded ? leftWandNode : rightWandNode;
		NiNode *offhandWandNode = isLeftHanded ? rightWandNode : leftWandNode;
		NiAVObject *rolloverNode = mainWandNode->GetObjectByName(&rolloverNodeStr.data);
		if (!rolloverNode) {
			rolloverNode = offhandWandNode->GetObjectByName(&rolloverNodeStr.data);
			mainWandNode->AttachChild(rolloverNode, false);
			offhandWandNode->RemoveChild(rolloverNode);
		}
		if (g_hasSavedRollover) {
			rolloverNode->m_localTransform = g_normalRolloverTransform;
		}

		if (g_hasSavedRumbleIntensity) {
			Setting	* activateRumbleIntensitySetting = GetINISetting("fActivateRumbleIntensity:VRInput");
			activateRumbleIntensitySetting->data.f32 = g_normalRumbleIntensity;
		}
	}

	return true;
}


void ControllerStateCB(uint32_t unControllerDeviceIndex, vr_src::VRControllerState001_t *pControllerState, uint32_t unControllerStateSize, bool& state)
{
	if (!initComplete) return;

	if (MenuChecker::isGameStopped()) return;

	vr_src::ETrackedControllerRole rightControllerRole = vr_src::ETrackedControllerRole::TrackedControllerRole_RightHand;
	vr_src::TrackedDeviceIndex_t rightController = (*g_openVR)->vrSystem->GetTrackedDeviceIndexForControllerRole(rightControllerRole);

	vr_src::ETrackedControllerRole leftControllerRole = vr_src::ETrackedControllerRole::TrackedControllerRole_LeftHand;
	vr_src::TrackedDeviceIndex_t leftController = (*g_openVR)->vrSystem->GetTrackedDeviceIndexForControllerRole(leftControllerRole);

	if (unControllerDeviceIndex == rightController) {
		g_rightGrabber->ControllerStateUpdate(unControllerDeviceIndex, pControllerState);
	}
	else if (unControllerDeviceIndex == leftController) {
		g_leftGrabber->ControllerStateUpdate(unControllerDeviceIndex, pControllerState);
	}
}


extern "C" {
	void OnDataLoaded()
	{
		const ModInfo *modInfo = DataHandler::GetSingleton()->LookupModByName("ForcePullVR.esp");
		if (!modInfo) {
			_ERROR("[CRITICAL] Could not get modinfo. Most likely the .esp is not loaded.");
			return;
		}

		TESForm *shaderForm = LookupFormByID(GetFullFormID(modInfo, 0x6F00));
		if (!shaderForm) {
			_ERROR("Failed to get slected item shader form");
			return;
		}
		g_itemSelectedShader = DYNAMIC_CAST(shaderForm, TESForm, TESEffectShader);
		if (!g_itemSelectedShader) {
			_ERROR("Failed to cast selected item shader form");
			return;
		}
		
		shaderForm = LookupFormByID(GetFullFormID(modInfo, 0x6F01));
		if (!shaderForm) {
			_ERROR("Failed to get slected item off limits shader form");
			return;
		}
		g_itemSelectedShaderOffLimits = DYNAMIC_CAST(shaderForm, TESForm, TESEffectShader);
		if (!g_itemSelectedShaderOffLimits) {
			_ERROR("Failed to cast selected item off limits shader form");
			return;
		}
		
		MenuManager * menuManager = MenuManager::GetSingleton();
		if (menuManager) {
			menuManager->MenuOpenCloseEventDispatcher()->AddEventSink(&MenuChecker::menuEvent);
		}

		// Init both grabbers

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

		g_shaderNodes = new std::unordered_map<ShaderReferenceEffect *, std::unordered_set<BSGeometry *>>;

		g_rightGrabber = new Grabber(false, "R", "NPC R Hand [RHnd]", "NPC R UpperArm [RUar]", "RightWandNode", rightFingerNames, { 0, -2.4, 6 }, { 7, -5, -2 }, Config::options.delayRightGripInput);
		g_leftGrabber = new Grabber(true, "L", "NPC L Hand [LHnd]", "NPC L UpperArm [LUar]", "LeftWandNode", leftFingerNames, { 0, -2.4, 6 }, { -7, -7, -3 }, Config::options.delayLeftGripInput);
		if (!g_rightGrabber || !g_leftGrabber || !g_shaderNodes) {
			_ERROR("[CRITICAL] Couldn't allocate memory");
			return;
		}

		g_rightGrabber->itemSelectedShader = g_itemSelectedShader;
		g_rightGrabber->itemSelectedShaderOffLimits = g_itemSelectedShaderOffLimits;

		g_leftGrabber->itemSelectedShader = g_itemSelectedShader;
		g_leftGrabber->itemSelectedShaderOffLimits = g_itemSelectedShaderOffLimits;

		// Right vector points to the right of the text
		g_rolloverRotation.data[0][0] = 0;
		g_rolloverRotation.data[1][0] = -cosf(30 * 0.0174533);
		g_rolloverRotation.data[2][0] = -sinf(30 * 0.0174533);
		// Forward vector points into the text
		g_rolloverRotation.data[0][1] = -1;
		g_rolloverRotation.data[1][1] = 0;
		g_rolloverRotation.data[2][1] = 0;
		// Up vector points up from the text
		g_rolloverRotation.data[0][2] = 0;
		g_rolloverRotation.data[1][2] = sinf(30 * 0.0174533);
		g_rolloverRotation.data[2][2] = -cosf(30 * 0.0174533);

		g_rightGrabber->rolloverRotation = g_rolloverRotation;
		g_rightGrabber->rolloverScale = Config::options.rolloverScale;

		// Flip right/forward vectors
		g_leftGrabber->rolloverRotation = g_rolloverRotation;
		g_leftGrabber->rolloverRotation.data[0][0] = -g_leftGrabber->rolloverRotation.data[0][0];
		g_leftGrabber->rolloverRotation.data[1][0] = -g_leftGrabber->rolloverRotation.data[1][0];
		g_leftGrabber->rolloverRotation.data[2][0] = -g_leftGrabber->rolloverRotation.data[2][0];
		g_leftGrabber->rolloverRotation.data[0][1] = -g_leftGrabber->rolloverRotation.data[0][1];
		g_leftGrabber->rolloverRotation.data[1][1] = -g_leftGrabber->rolloverRotation.data[1][1];
		g_leftGrabber->rolloverRotation.data[2][1] = -g_leftGrabber->rolloverRotation.data[2][1];
		g_leftGrabber->rolloverScale = Config::options.rolloverScale;

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
			else if (msg->type == SKSEMessagingInterface::kMessage_PreLoadGame) {
				_MESSAGE("SKSE PreLoadGame message received");
				g_isLoaded = false;
			}
			else if (msg->type == SKSEMessagingInterface::kMessage_PostLoadGame || msg->type == SKSEMessagingInterface::kMessage_NewGame) {
				_MESSAGE("SKSE PostLoadGame or NewGame message received, type: %d", msg->type);
				g_isLoaded = true;
			}
			else if (msg->type == SKSEMessagingInterface::kMessage_PostLoad) {
				// Get the VRIK plugin API
				g_vrikInterface = vrikPluginApi::getVrikInterface001(g_pluginHandle, g_messaging);
			}
		}
	}

	bool SKSEPlugin_Query(const SKSEInterface* skse, PluginInfo* info)
	{
		gLog.OpenRelative(CSIDL_MYDOCUMENTS, "\\My Games\\Skyrim VR\\SKSE\\ForcePullVR.log");
		gLog.SetPrintLevel(IDebugLog::kLevel_DebugMessage);
		gLog.SetLogLevel(IDebugLog::kLevel_DebugMessage);

		_MESSAGE("ForcePullVR v%s", FPVR_VERSION_VERSTRING);

		info->infoVersion = PluginInfo::kInfoVersion;
		info->name = "ForcePullVR";
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
		_MESSAGE("ForcePullVR loaded");

		if (Config::ReadConfigOptions()) {
			_MESSAGE("Successfully read config parameters");
		}
		else {
			_WARNING("[WARNING] Failed to read config options. Using defaults instead.");
		}

		_MESSAGE("Registering for SKSE messages");
		g_messaging = (SKSEMessagingInterface*)skse->QueryInterface(kInterface_Messaging);
		g_messaging->RegisterListener(g_pluginHandle, "SKSE", OnSKSEMessage);

		g_vrInterface = (SKSEVRInterface *)skse->QueryInterface(kInterface_VR);
		if (!g_vrInterface) {
			_ERROR("[CRITICAL] Couldn't get SKSE VR interface. You probably have an outdated SKSE version.");
			return false;
		}
		g_vrInterface->RegisterForControllerState(g_pluginHandle, 0, ControllerStateCB);
		g_vrInterface->RegisterForPoses(g_pluginHandle, 0, WaitPosesCB);

		g_trampoline = (SKSETrampolineInterface *)skse->QueryInterface(kInterface_Trampoline);
		if (!g_trampoline) {
			_WARNING("Couldn't get trampoline interface");
		}
		if (!TryHook()) {
			_ERROR("[CRITICAL] Failed to perform hooks");
			return false;
		}

		g_timer.Start();

		return true;
	}
};
