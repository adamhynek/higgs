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
#include "xbyak/xbyak.h"

#include <ShlObj.h>  // CSIDL_MYDOCUMENTS

#include "grabber.h"
#include "version.h"
#include "utils.h"
#include "config.h"
#include "menu_checker.h"
#include "shaders.h"
#include "offsets.h"


// SKSE globals
static PluginHandle	g_pluginHandle = kPluginHandle_Invalid;
static SKSEMessagingInterface *g_messaging = nullptr;

SKSEVRInterface *g_vrInterface = nullptr;
SKSETrampolineInterface *g_trampoline = nullptr;

NiMatrix33 g_rolloverRotation; // Set on plugin load

BSFixedString rolloverNodeStr("WSActivateRollover");

bool g_isLoaded = false;

TESEffectShader *g_itemSelectedShader = nullptr;
TESEffectShader *g_itemSelectedShaderOffLimits = nullptr;

bool g_hasSavedRumbleIntensity = false;
float g_normalRumbleIntensity;
bool g_hasSavedRollover = false;
NiTransform g_normalRolloverTransform;

Grabber *g_rightGrabber;
Grabber *g_leftGrabber;


auto shaderHookLoc = RelocAddr<uintptr_t>(0x2AE3E8);
auto shaderHookedFunc = RelocAddr<uintptr_t>(0x564DD0);

uintptr_t shaderHookedFuncAddr = 0;

auto worldHookLoc = RelocAddr<uintptr_t>(0x271EF9);


ShaderReferenceEffect ** volatile g_shaderReferenceToSet = nullptr;
void HookedShaderReferenceEffectCtor(ShaderReferenceEffect *ref) // DO NOT USE THIS MEMORY YET - IT HAS JUST BEEN ALLOCATED. LET THE GAME CONSTRUCT IT IN A BIT.
{
	if (g_shaderReferenceToSet) {
		*g_shaderReferenceToSet = ref;
	}
}

void HookedWorldUpdateHook(bhkWorld *world)
{
	//_MESSAGE("Pre World update hook");
	// Perform the same operation both in this hook and in the main thread.
	// Why? We need it here to calm down physics constraints - they freak out if only set in the openvr hook
	// We also need to do it there though, since otherwise we get flickering lighting on the object.
	{
		std::lock_guard<std::mutex> lock(g_rightGrabber->deselectLock);

		if (g_rightGrabber->state == Grabber::State_Held) {
			NiAVObject *handNode = (*g_thePlayer)->GetNiRootNode(1)->GetObjectByName(&g_rightGrabber->handNodeName.data);
			if (handNode) {
				NiPointer<TESObjectREFR> selectedObj;
				if (LookupREFRByHandle(g_rightGrabber->selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {
					NiAVObject *n = FindCollidableNode(g_rightGrabber->selectedObject.collidable);
					if (n) {
						NiTransform inverseParent;
						n->m_parent->m_worldTransform.Invert(inverseParent);
						NiTransform newTransform = handNode->m_worldTransform * g_rightGrabber->initialObjTransformHandSpace;
						n->m_localTransform = inverseParent * newTransform;
						NiAVObject::ControllerUpdateContext ctx;
						ctx.flags = 0x2000; // makes havok sim more stable?
						ctx.delta = 0;
						NiAVObject_UpdateObjectUpwards(n, &ctx);
					}
				}
			}
		}
	}
	{
		std::lock_guard<std::mutex> lock(g_leftGrabber->deselectLock);

		if (g_leftGrabber->state == Grabber::State_Held) {
			NiAVObject *handNode = (*g_thePlayer)->GetNiRootNode(1)->GetObjectByName(&g_leftGrabber->handNodeName.data);
			if (handNode) {
				NiPointer<TESObjectREFR> selectedObj;
				if (LookupREFRByHandle(g_leftGrabber->selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {
					NiAVObject *n = FindCollidableNode(g_leftGrabber->selectedObject.collidable);
					if (n) {
						NiTransform inverseParent;
						n->m_parent->m_worldTransform.Invert(inverseParent);
						NiTransform newTransform = handNode->m_worldTransform * g_leftGrabber->initialObjTransformHandSpace;
						n->m_localTransform = inverseParent * newTransform;
						NiAVObject::ControllerUpdateContext ctx;
						ctx.flags = 0x2000; // makes havok sim more stable?
						ctx.delta = 0;
						NiAVObject_UpdateObjectUpwards(n, &ctx);
					}
				}
			}
		}
	}
}


void Hook_Commit(void)
{
	{
		struct ShaderCode : Xbyak::CodeGenerator {
			ShaderCode(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				push(rax);
				push(rcx);
				push(rdx);
				push(r8);
				push(r9);
				push(r10);
				push(r11);
				sub(rsp, 0x68); // Need to keep the stack SIXTEEN BYTE ALIGNED
				movsd(ptr[rsp], xmm0);
				movsd(ptr[rsp + 0x10], xmm1);
				movsd(ptr[rsp + 0x20], xmm2);
				movsd(ptr[rsp + 0x30], xmm3);
				movsd(ptr[rsp + 0x40], xmm4);
				movsd(ptr[rsp + 0x50], xmm5);

				// Call our hook
				mov(rax, (uintptr_t)HookedShaderReferenceEffectCtor);
				call(rax);

				movsd(xmm0, ptr[rsp]);
				movsd(xmm1, ptr[rsp + 0x10]);
				movsd(xmm2, ptr[rsp + 0x20]);
				movsd(xmm3, ptr[rsp + 0x30]);
				movsd(xmm4, ptr[rsp + 0x40]);
				movsd(xmm5, ptr[rsp + 0x50]);
				add(rsp, 0x68);
				pop(r11);
				pop(r10);
				pop(r9);
				pop(r8);
				pop(rdx);
				pop(rcx);
				pop(rax);

				// Original code
				mov(rax, shaderHookedFuncAddr);
				call(rax);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(shaderHookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		ShaderCode code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(shaderHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("Shader hook complete");
	}

	{
		struct WorldCode : Xbyak::CodeGenerator {
			WorldCode(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				push(rax);
				push(rcx);
				push(rdx);
				push(r8);
				push(r9);
				push(r10);
				push(r11);
				sub(rsp, 0x68); // Need to keep the stack SIXTEEN BYTE ALIGNED
				movsd(ptr[rsp], xmm0);
				movsd(ptr[rsp + 0x10], xmm1);
				movsd(ptr[rsp + 0x20], xmm2);
				movsd(ptr[rsp + 0x30], xmm3);
				movsd(ptr[rsp + 0x40], xmm4);
				movsd(ptr[rsp + 0x50], xmm5);

				// Call our hook
				mov(rax, (uintptr_t)HookedWorldUpdateHook);
				call(rax);

				movsd(xmm0, ptr[rsp]);
				movsd(xmm1, ptr[rsp + 0x10]);
				movsd(xmm2, ptr[rsp + 0x20]);
				movsd(xmm3, ptr[rsp + 0x30]);
				movsd(xmm4, ptr[rsp + 0x40]);
				movsd(xmm5, ptr[rsp + 0x50]);
				add(rsp, 0x68);
				pop(r11);
				pop(r10);
				pop(r9);
				pop(r8);
				pop(rdx);
				pop(rcx);
				pop(rax);

				// Original code
				call(ptr[r8 + 0x190]);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(worldHookLoc.GetUIntPtr() + 7);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		WorldCode code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(worldHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("World update hook complete");
	}
}



bool TryHook()
{
	// This should be sized to the actual amount used by your trampoline
	static const size_t TRAMPOLINE_SIZE = 512;

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

	Hook_Commit();
	return true;
}


bool WaitPosesCB(vr_src::TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, vr_src::TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount)
{
	if (!g_isLoaded) return true;

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

	std::pair<bool, bool> validItems = { true, true };
	if (!Config::options.ignoreWeaponChecks) {
		validItems = AreEquippedItemsValid(player);
	}

	bool isLeftHanded = *g_leftHandedMode;

	g_rightGrabber->PoseUpdate(*g_leftGrabber, isLeftHanded ? validItems.second : validItems.first, playerWorldNode);
	g_leftGrabber->PoseUpdate(*g_rightGrabber, isLeftHanded ? validItems.first : validItems.second, playerWorldNode);

	if (g_rightGrabber->IsSafeToClearSavedCollision() && g_leftGrabber->IsSafeToClearSavedCollision()) {
		// cleanup the collision id map to prevent mem leaks when an item is destroyed (i.e. 'activated', etc.) while holding / pulling it
		ClearCollisionMap();
	}

	bool displayLeft = g_leftGrabber->ShouldDisplayRollover();
	bool displayRight = g_rightGrabber->ShouldDisplayRollover();

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
			if (g_leftGrabber->selectionLockedTime > g_rightGrabber->selectionLockedTime) {
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
		// Can't set these statically as the invalidrefhandle doesn't load until later
		g_rightGrabber->selectedObject.handle = *g_invalidRefHandle;
		g_leftGrabber->selectedObject.handle = *g_invalidRefHandle;

		const ModInfo *modInfo = DataHandler::GetSingleton()->LookupModByName("ForcePullVR.esp");
		if (!modInfo) {
			_MESSAGE("[CRITICAL] Could not get modinfo. Most likely the .esp is not loaded.");
			return;
		}

		TESForm *shaderForm = LookupFormByID(GetFullFormID(modInfo, 0x6F00));
		if (!shaderForm) {
			_MESSAGE("Failed to get slected item shader form");
			return;
		}
		g_itemSelectedShader = DYNAMIC_CAST(shaderForm, TESForm, TESEffectShader);
		if (!g_itemSelectedShader) {
			_MESSAGE("Failed to cast selected item shader form");
			return;
		}
		
		shaderForm = LookupFormByID(GetFullFormID(modInfo, 0x6F01));
		if (!shaderForm) {
			_MESSAGE("Failed to get slected item off limits shader form");
			return;
		}
		g_itemSelectedShaderOffLimits = DYNAMIC_CAST(shaderForm, TESForm, TESEffectShader);
		if (!g_itemSelectedShaderOffLimits) {
			_MESSAGE("Failed to cast selected item off limits shader form");
			return;
		}
		
		MenuManager * menuManager = MenuManager::GetSingleton();
		if (menuManager) {
			menuManager->MenuOpenCloseEventDispatcher()->AddEventSink(&MenuChecker::menuEvent);
		}

		g_rightGrabber->itemSelectedShader = g_itemSelectedShader;
		g_rightGrabber->itemSelectedShaderOffLimits = g_itemSelectedShaderOffLimits;

		g_leftGrabber->itemSelectedShader = g_itemSelectedShader;
		g_leftGrabber->itemSelectedShaderOffLimits = g_itemSelectedShaderOffLimits;

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

		g_rightGrabber = new Grabber("R", "NPC R Hand [RHnd]", "NPC R UpperArm [RUar]", "RightWandNode", "AnimObjectR", { 7, -5, -2 }, Config::options.delayRightGripInput);
		g_leftGrabber = new Grabber("L", "NPC L Hand [LHnd]", "NPC L UpperArm [LUar]", "LeftWandNode", "AnimObjectL", { -7, -7, -3 }, Config::options.delayLeftGripInput);
		if (!g_rightGrabber || !g_leftGrabber) {
			_ERROR("[CRITICAL] Couldn't allocate memory");
			return false;
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

		shaderHookedFuncAddr = shaderHookedFunc.GetUIntPtr(); // before trampolines

		g_trampoline = (SKSETrampolineInterface *)skse->QueryInterface(kInterface_Trampoline);
		if (!g_trampoline) {
			_ERROR("couldn't get trampoline interface");
		}
		if (!TryHook()) {
			_ERROR("Failed to perform hook");
		}

		g_timer.Start();

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

		return true;
	}
};
