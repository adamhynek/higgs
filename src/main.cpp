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

Hand *g_rightHand = nullptr;
Hand *g_leftHand = nullptr;

std::unordered_map<ShaderReferenceEffect *, std::unordered_set<BSGeometry *>> *g_shaderNodes;

PlayingShader *g_playingShaders; // size == 2
std::unordered_map<NiAVObject *, NiPointer<ShaderReferenceEffect>> *g_effectDataMap;

ContactListener *contactListener = nullptr;
IslandDeactivationListener activationListener;

int g_savedShadowUpdateFrameDelay = -1;
int g_shadowUpdateFrame = 0;
int g_numShadowUpdates = 0;


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


void FillControllerVelocities(NiAVObject *hmdNode, vr_src::TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount)
{
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
								g_rightHand->controllerVelocities.pop_back();
								g_rightHand->controllerVelocities.push_front(velocityWorldspace);

								NiPoint3 openvrAngularVelocity = { pose.vAngularVelocity.v[0], pose.vAngularVelocity.v[1], pose.vAngularVelocity.v[2] };
								NiPoint3 skyrimAngularVelocity = { openvrAngularVelocity.x, -openvrAngularVelocity.z, openvrAngularVelocity.y };
								NiPoint3 angularVelocityWorldspace = openvrToSkyrimWorldTransform * skyrimAngularVelocity;
								g_rightHand->controllerAngularVelocities.pop_back();
								g_rightHand->controllerAngularVelocities.push_front(angularVelocityWorldspace);
							}
						}
						else if (i == leftIndex && isLeftConnected) {
							vr_src::TrackedDevicePose_t &pose = pGamePoseArray[i];
							if (pose.bDeviceIsConnected && pose.bPoseIsValid && pose.eTrackingResult == vr_src::ETrackingResult::TrackingResult_Running_OK) {
								NiPoint3 openvrVelocity = { pose.vVelocity.v[0], pose.vVelocity.v[1], pose.vVelocity.v[2] };
								NiPoint3 skyrimVelocity = { openvrVelocity.x, -openvrVelocity.z, openvrVelocity.y };
								NiPoint3 velocityWorldspace = openvrToSkyrimWorldTransform * skyrimVelocity;
								g_leftHand->controllerVelocities.pop_back();
								g_leftHand->controllerVelocities.push_front(velocityWorldspace);

								NiPoint3 openvrAngularVelocity = { pose.vAngularVelocity.v[0], pose.vAngularVelocity.v[1], pose.vAngularVelocity.v[2] };
								NiPoint3 skyrimAngularVelocity = { openvrAngularVelocity.x, -openvrAngularVelocity.z, openvrAngularVelocity.y };
								NiPoint3 angularVelocityWorldspace = openvrToSkyrimWorldTransform * skyrimAngularVelocity;
								g_leftHand->controllerAngularVelocities.pop_back();
								g_leftHand->controllerAngularVelocities.push_front(angularVelocityWorldspace);
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


void Update()
{
	UpdateShadowDelay();

	if (!initComplete) return;

	if (MenuChecker::isGameStopped()) return;

	PlayerCharacter *player = *g_thePlayer;
	if (!player || !player->GetNiNode()) return;

	//Config::ReloadIfModified(); // TODO: Remove

	g_currentFrameTime = GetTime();

	NiPointer<NiAVObject> playerWorldObj = player->unk3F0[PlayerCharacter::Node::kNode_PlayerWorldNode];
	NiNode *playerWorldNode = playerWorldObj ? playerWorldObj->GetAsNiNode() : nullptr;
	if (!playerWorldNode) return;

	NiPointer<NiAVObject> rightWandObj = player->unk3F0[PlayerCharacter::Node::kNode_RightWandNode];
	NiNode *rightWandNode = rightWandObj ? rightWandObj->GetAsNiNode() : nullptr;
	if (!rightWandNode) return;

	NiPointer<NiAVObject> leftWandObj = player->unk3F0[PlayerCharacter::Node::kNode_LeftWandNode];
	NiNode *leftWandNode = leftWandObj ? leftWandObj->GetAsNiNode() : nullptr;
	if (!leftWandNode) return;

	TESObjectCELL *cell = player->parentCell;
	if (!cell) return;

	NiPointer<bhkWorld> world = GetHavokWorldFromCell(cell);
	if (!world) {
		_MESSAGE("Could not get havok world from player cell");
		return;
	}

	if (world != contactListener->world) {
		bhkWorld *oldWorld = contactListener->world;
		if (oldWorld) {
			// If exists, remove the listener from the previous world
			_MESSAGE("Removing listeners and collision from old havok world");

			{
				BSWriteLocker lock(&oldWorld->worldLock);

				hkpWorld_removeContactListener(oldWorld->world, contactListener);
				if (Config::options.enableShadowUpdateFix) {
					hkpWorld_removeIslandActivationListener(oldWorld->world, &activationListener);
				}

				g_rightHand->RemoveHandCollision(oldWorld);
				g_leftHand->RemoveHandCollision(oldWorld);

				g_rightHand->RemoveWeaponCollision(oldWorld);
				g_leftHand->RemoveWeaponCollision(oldWorld);
			}
		}

		_MESSAGE("Adding listeners and collision to new havok world");

		{
			BSWriteLocker lock(&world->worldLock);

			AddCustomCollisionLayer(world);

			hkpWorld_addContactListener(world->world, contactListener);
			if (Config::options.enableShadowUpdateFix) {
				hkpWorld_addIslandActivationListener(world->world, &activationListener);
			}

			g_rightHand->CreateHandCollision(world);
			g_leftHand->CreateHandCollision(world);

			g_rightHand->CreateWeaponCollision(world);
			g_leftHand->CreateWeaponCollision(world);
		}

		contactListener->world = world;
	}

	bool isLeftHanded = *g_leftHandedMode;

	std::pair<bool, bool> validItems = AreEquippedItemsValid(player);
	bool isRightValid = isLeftHanded ? validItems.second : validItems.first;
	bool isLeftValid = isLeftHanded ? validItems.first : validItems.second;

	isRightValid &= !g_interface001.IsDisabled(false);
	isLeftValid &= !g_interface001.IsDisabled(true);

	bool isRightHeld = g_rightHand->HasHeldKeyframed();
	bool isLeftHeld = g_leftHand->HasHeldKeyframed();

	Hand *firstHandToUpdate = g_rightHand;
	Hand *lastHandToUpdate = g_leftHand;
	bool isFirstValid = isRightValid;
	bool isLastValid = isLeftValid;
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
					isFirstValid = isLeftValid;
					lastHandToUpdate = g_rightHand;
					isLastValid = isRightValid;
				}
				else if (DoesNodeHaveNode(rightNode, leftNode)) {
					// Left is the child
					firstHandToUpdate = g_rightHand;
					isFirstValid = isRightValid;
					lastHandToUpdate = g_leftHand;
					isLastValid = isLeftValid;
				}
			}
		}
	}

	firstHandToUpdate->Update(*lastHandToUpdate, isFirstValid, playerWorldNode, world);
	lastHandToUpdate->Update(*firstHandToUpdate, isLastValid, playerWorldNode, world);

	if (g_rightHand->IsSafeToClearSavedCollision() && g_leftHand->IsSafeToClearSavedCollision()) {
		// cleanup the collision id map to prevent mem leaks when an item is destroyed (i.e. 'activated', etc.) while holding / pulling it
		CollisionInfo::ClearCollisionMap();
	}
}


bool WaitPosesCB(vr_src::TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, vr_src::TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount)
{
	PlayerCharacter *player = *g_thePlayer;
	if (!player || !player->GetNiNode()) return true;

	NiPointer<NiAVObject> hmdNode = player->unk3F0[PlayerCharacter::Node::kNode_HmdNode];
	if (!hmdNode) return true;

	FillControllerVelocities(hmdNode, pGamePoseArray, unGamePoseArrayCount);

	//Update();

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

	bool isLeftHanded = *g_leftHandedMode;

	std::pair<bool, bool> validItems = AreEquippedItemsValid(player);
	bool isRightValid = isLeftHanded ? validItems.second : validItems.first;
	bool isLeftValid = isLeftHanded ? validItems.first : validItems.second;

	if (unControllerDeviceIndex == rightController) {
		g_rightHand->ControllerStateUpdate(unControllerDeviceIndex, pControllerState, isRightValid);
	}
	else if (unControllerDeviceIndex == leftController) {
		g_leftHand->ControllerStateUpdate(unControllerDeviceIndex, pControllerState, isLeftValid);
	}
}


void ShowErrorBox(const char *errorString)
{
	_ERROR(errorString);
	int msgboxID = MessageBox(
		NULL,
		(LPCTSTR)errorString,
		(LPCTSTR)"HIGGS Fatal Error",
		MB_ICONERROR | MB_OK | MB_TASKMODAL
	);
}

void ShowErrorBoxAndTerminate(const char *errorString)
{
	ShowErrorBox(errorString);
	*((int *)0) = 0xDEADBEEF; // crash
}

struct ShowErrorBoxAndTerminateTask : TaskDelegate
{
	static ShowErrorBoxAndTerminateTask * Create(const char *errorString) {
		ShowErrorBoxAndTerminateTask * cmd = new ShowErrorBoxAndTerminateTask;
		if (cmd)
		{
			cmd->errorString = errorString;
		}
		return cmd;
	}
	virtual void Run() {
		ShowErrorBoxAndTerminate(errorString);
	}
	virtual void Dispose() {
		delete this;
	}

	const char *errorString;
};

void QueueErrorBox(const char *errorString)
{
	// Show error box and terminate on main thread
	g_taskInterface->AddTask(ShowErrorBoxAndTerminateTask::Create(errorString));
}


extern "C" {
	void OnDataLoaded()
	{
		const ModInfo *modInfo = DataHandler::GetSingleton()->LookupModByName("higgs_vr.esp");
		if (!modInfo) {
			QueueErrorBox("[CRITICAL] Could not get modinfo. Most likely the higgs .esp doesn't exist.");
			return;
		}

		if (!modInfo->IsActive()) {
			QueueErrorBox("[CRITICAL] The higgs .esp exists, but is not active. Make sure the esp is enabled in your mod manager.");
			return;
		}

		TESForm *shaderForm = LookupFormByID(GetFullFormID(modInfo, 0x6F00));
		if (!shaderForm) {
			QueueErrorBox("Failed to get slected item shader form");
			return;
		}
		g_itemSelectedShader = DYNAMIC_CAST(shaderForm, TESForm, TESEffectShader);
		if (!g_itemSelectedShader) {
			QueueErrorBox("Failed to cast selected item shader form");
			return;
		}
		
		shaderForm = LookupFormByID(GetFullFormID(modInfo, 0x6F01));
		if (!shaderForm) {
			QueueErrorBox("Failed to get slected item off limits shader form");
			return;
		}
		g_itemSelectedShaderOffLimits = DYNAMIC_CAST(shaderForm, TESForm, TESEffectShader);
		if (!g_itemSelectedShaderOffLimits) {
			QueueErrorBox("Failed to cast selected item off limits shader form");
			return;
		}
		
		MenuManager * menuManager = MenuManager::GetSingleton();
		if (menuManager) {
			menuManager->MenuOpenCloseEventDispatcher()->AddEventSink(&MenuChecker::menuEvent);
		}

		g_isVrikPresent = GetModuleHandle("vrik") != NULL;

		// Need to heap-allocate and "leak" anything with NiPointers since if they're statically allocated we crash when the game exits and these objects destruct
		g_shaderNodes = new std::unordered_map<ShaderReferenceEffect *, std::unordered_set<BSGeometry *>>;

		g_playingShaders = new PlayingShader[2];
		g_effectDataMap = new std::unordered_map<NiAVObject *, NiPointer<ShaderReferenceEffect>>;

		contactListener = new ContactListener;

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

		g_rightHand = new Hand(false, "R", "NPC R Hand [RHnd]", "RightWandNode", rightFingerNames, rightPalm, Config::options.rolloverOffsetRight, Config::options.delayRightGripInput);
		g_leftHand = new Hand(true, "L", "NPC L Hand [LHnd]", "LeftWandNode", leftFingerNames, leftPalm, Config::options.rolloverOffsetLeft, Config::options.delayLeftGripInput);
		if (!g_rightHand || !g_leftHand || !g_shaderNodes) {
			QueueErrorBox("[CRITICAL] Couldn't allocate memory");
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

		if (Config::options.disableRolloverRumble) {
			_MESSAGE("Disabling rollover rumble");
			Setting	* activateRumbleIntensitySetting = GetINISetting("fActivateRumbleIntensity:VRInput");
			activateRumbleIntensitySetting->SetDouble(0);
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
				}
				else {
					_MESSAGE("Did not get VRIK api");
				}
			}
		}
	}

	bool SKSEPlugin_Query(const SKSEInterface* skse, PluginInfo* info)
	{
		gLog.OpenRelative(CSIDL_MYDOCUMENTS, "\\My Games\\Skyrim VR\\SKSE\\higgs_vr.log");
		gLog.SetPrintLevel(IDebugLog::kLevel_DebugMessage);
		gLog.SetLogLevel(IDebugLog::kLevel_DebugMessage);

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

		_MESSAGE("Registering for SKSE messages");
		g_messaging = (SKSEMessagingInterface*)skse->QueryInterface(kInterface_Messaging);
		g_messaging->RegisterListener(g_pluginHandle, "SKSE", OnSKSEMessage);

		g_vrInterface = (SKSEVRInterface *)skse->QueryInterface(kInterface_VR);
		if (!g_vrInterface) {
			ShowErrorBox("[CRITICAL] Couldn't get SKSE VR interface. You probably have an outdated SKSE version.");
			return false;
		}
		g_vrInterface->RegisterForControllerState(g_pluginHandle, 66, ControllerStateCB);
		g_vrInterface->RegisterForPoses(g_pluginHandle, 66, WaitPosesCB);

		g_papyrus = (SKSEPapyrusInterface *)skse->QueryInterface(kInterface_Papyrus);
		if (!g_papyrus) {
			ShowErrorBox("[CRITICAL] Couldn't get Papyrus interface");
			return false;
		}
		if (g_papyrus->Register(PapyrusAPI::RegisterPapyrusFuncs)) {
			_MESSAGE("Successfully registered papyrus functions");
		}

		g_taskInterface = (SKSETaskInterface *)skse->QueryInterface(kInterface_Task);
		if (!g_taskInterface) {
			ShowErrorBox("[CRITICAL] Could not get SKSE task interface");
			return false;
		}

		g_trampoline = (SKSETrampolineInterface *)skse->QueryInterface(kInterface_Trampoline);
		if (!g_trampoline) {
			_WARNING("Couldn't get trampoline interface");
		}
		if (!TryHook()) {
			ShowErrorBox("[CRITICAL] Failed to perform hooks");
			return false;
		}

		g_timer.Start();

		return true;
	}
};
