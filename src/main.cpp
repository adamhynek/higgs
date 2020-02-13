#include <functional>
#include <string>
#include <regex>
#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>
#include "common/IDebugLog.h"  // IDebugLog
#include "skse64_common/skse_version.h"  // RUNTIME_VERSION
#include "skse64/PluginAPI.h"  // SKSEInterface, PluginInfo
#include "skse64/GameRTTI.h"
#include "skse64/GameSettings.h"
#include "skse64/NiNodes.h"
#include "skse64/NiExtraData.h"
#include "skse64/GameData.h"
#include "skse64/GameForms.h"
#include "skse64/PapyrusActor.h"

#include <ShlObj.h>  // CSIDL_MYDOCUMENTS

#include "main.h"
#include "version.h"
#include "physics.h"
#include "utils.h"

// Headers under api/ folder
#include "api/PapyrusVRAPI.h"
#include "api/VRManagerAPI.h"
#include "api/utils/OpenVRUtils.h"


// Multiply skyrim coords by this to get havok coords
// It's the number of meters per skyrim unit
RelocAddr<float *> HAVOK_WORLD_SCALE_ADDR(0x15B78F4);

// Address of pointer that points to the bhkWorld pointer
RelocAddr<bhkWorld ***> BHKWORLD(0x1f850d0);

// Alternatively, 0x30008E0 + 0x78
// Even better, (*0x2FC60C0) + 0x78
// Address of pointer to bhkSimpleShapePhantom that tracks the right hand
RelocAddr<bhkSimpleShapePhantom **> SPHERE_SHAPE_ADDR(0x3000958);


// SKSE / SkyrimVRTools globals
static PluginHandle	g_pluginHandle = kPluginHandle_Invalid;
static SKSEMessagingInterface *g_messaging = nullptr;

PapyrusVRAPI *g_papyrusvr = nullptr;
PapyrusVR::VRManagerAPI *g_papyrusvrManager = nullptr;
OpenVRHookManagerAPI *g_openvrHook = nullptr;

VMClassRegistry *vmRegistry = nullptr;

// Gets callbacks from havok linear cast
CdPointCollector cdPointCollector;
__declspec(align(16)) hkpLinearCastInput linearCastInput; // Need to be 128-bit aligned for simd hkVector4 types, or we CRASH

TESObjectREFR *pullObj = nullptr;
TESObjectREFR *prevPullObj = nullptr;

NiPoint3 initialPullObjRelativePosition(0, 0, 0);
NiPoint3 prevHandPosLocal(0, 0, 0); // Relative to hmd

bool pullDesired = false;
bool pushDesired = false;
bool shaderApplied = false;

long long lastDebugCastTime = 0;

bool isLoaded = false;
bool isLastUpdateValid = false;

//UInt32 spellFormId = 0x1C789; // fireball
//UInt32 spellFormId = 0x2B96C; // ice spike
//UInt32 spellFormId = 0x2dd29; // lightning bolt
//UInt32 spellFormId = 0x00012fcd; // flames
//UInt32 spellFormId = 0x0001A4CC; // telekinesis
SpellItem *g_debugSpell = nullptr;
TESObjectREFR *debugSourceActivator = nullptr;
TESObjectREFR *debugTargetActivator = nullptr;
bool g_debug = false; // Set to true to fire a spell (visual only) in the direction that the cast happens

TESEffectShader *g_itemSelectedShader = nullptr;


// Called on each update (about 90-100 calls per second)
void OnPoseUpdate(float deltaTime)
{
	if (!isLoaded) return;

	if (IsInMenuMode(vmRegistry, 0)) return;

	PlayerCharacter *player = *g_thePlayer;
	if (!player || !player->loadedState || !player->loadedState->node)
		return;

	TESObjectCELL* cell = player->parentCell;
	if (!cell)
		return;

	bool wasLastUpdateValid = isLastUpdateValid;
	isLastUpdateValid = false;


	long long currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();


	static BSFixedString rHandStr("NPC R Hand [RHnd]");
	NiAVObject *rightHand = player->GetNiRootNode(1)->GetObjectByName(&rHandStr.data);
	if (!rightHand) {
		_MESSAGE("No right hand");
		return;
	}
	NiPoint3 handPos = rightHand->m_worldTransform.pos;

	NiPoint3 handAdjust = { -0.2, -1, 0.4 }; // A bit down (negative up), a lot left (negative right) and a bit forward
	handAdjust = VectorNormalized(handAdjust);
	NiPoint3 castDirection = rightHand->m_worldTransform.rot * handAdjust;

	static BSFixedString hmdNodeStr("HmdNode");
	NiAVObject *hmdNode = GetHighestParent(player->loadedState->node)->GetObjectByName(&hmdNodeStr.data);

	NiTransform inversePlayerTransform;
	hmdNode->m_worldTransform.Invert(inversePlayerTransform);

	NiPoint3 handPosLocal = inversePlayerTransform * handPos;

	static BSFixedString rClavicleStr("NPC R Clavicle [RClv]");
	NiAVObject *rightClavicle = player->GetNiRootNode(0)->GetObjectByName(&rClavicleStr.data);
	// TODO: Use delta between consecutive clavicle->hand vectors for hand motions?

	
	// Fire a spell at the direction that the cast happens, so that you can visually see and adjust
	if (g_debug) {
		UInt32 nullHandle = *g_invalidRefHandle;
		NiPoint3 rot(0, 0, 0);
		MoveRefrToPosition(debugSourceActivator, &nullHandle, player->parentCell, CALL_MEMBER_FN(player, GetWorldspace)(), &handPos, &rot);

		NiPoint3 targetPos = handPos + castDirection;
		MoveRefrToPosition(debugTargetActivator, &nullHandle, player->parentCell, CALL_MEMBER_FN(player, GetWorldspace)(), &targetPos, &rot);
		if (currentTime - lastDebugCastTime > 500) { // in ms
			lastDebugCastTime = currentTime;
			Cast(vmRegistry, 0, g_debugSpell, debugSourceActivator, debugTargetActivator);
		}
	}

	// Convert hand position from skyrim coords to havok coords
	float havokWorldScale = *HAVOK_WORLD_SCALE_ADDR;
	NiPoint3 hkHandPos = handPos * havokWorldScale;
	NiPoint3 hkTargetPos = hkHandPos + castDirection * 5;

	bhkWorld *world = **BHKWORLD;
	bhkSimpleShapePhantom *sphere = *SPHERE_SHAPE_ADDR;
	cdPointCollector.reset();
	float radiusBefore = sphere->phantom->m_motionState.m_objectRadius; // save radius so we can restore it
	sphere->phantom->m_motionState.m_objectRadius = 0.2f;
	// TODO: Maybe create our own phantom and add it to the world instead of reusing the game's one?
	// TODO: The cast sometimes goes through objects. I think this is because the sphere's position does not exactly match the hand's.
	linearCastInput.m_to = { hkTargetPos.x, hkTargetPos.y, hkTargetPos.z, 0 };
	hkpWorld_LinearCast(world->world, &sphere->phantom->m_collidable, &linearCastInput, &cdPointCollector, nullptr);
	sphere->phantom->m_motionState.m_objectRadius = radiusBefore;

	// Process result of cast
	// TODO: Search multiple cells around the player instead of just the one. Then it might be good to cache collision objs on cell change.
	// TODO: Measure how long these casts + searches take.
	for (int i = 0; i < cell->refData.maxSize; i++) {
		auto ref = cell->refData.refArray[i];
		if (ref.unk08 != nullptr && ref.ref) {
			auto obj = ref.ref;
			if (obj && obj->loadedState && obj->loadedState->node && obj->loadedState->node->unk040) {
				auto collisionObj = (bhkCollisionObject *)obj->loadedState->node->unk040;
				if (&collisionObj->body->hkBody->m_collidable == cdPointCollector.m_closestCollidable) {
					TESForm *baseForm = obj->baseForm;
					if (baseForm && baseForm->formType == kFormType_Weapon) {
						if (!shaderApplied) {
							EffectShader_Play(vmRegistry, 0, g_itemSelectedShader, obj, -1.0f);
							shaderApplied = true;
						}
						pullObj = obj;
					}
				}
			}
		}
	}

	if (pullObj && !(pullObj->flags & TESForm::kFlagIsDeleted) && pullObj->loadedState && pullObj->loadedState->node) {
		auto relObjPos = pullObj->pos - handPos;
		TESForm *baseForm = pullObj->baseForm;
		if (baseForm && baseForm->formType == kFormType_Weapon) {
			auto *weapon = DYNAMIC_CAST(baseForm, TESForm, TESObjectWEAP);
			if (weapon) {
				if (VectorLength(relObjPos) < 30) {
					_MESSAGE("Equipping");
					Activate(vmRegistry, 0, pullObj, player, false);
					papyrusActor::EquipItemEx(player, weapon, 1, false, false);
					pullObj = nullptr;
					pullDesired = false;
					pushDesired = false;
					shaderApplied = false;
				}
			}
		}

		if (pullObj) { // (Could be nulled out above)
			if (!prevPullObj) {
				initialPullObjRelativePosition = relObjPos;
			}

			// Determine the position where we want the object to be
			float theta = asinf(castDirection.z); // vertical angle of hand relative to horizontal
			float w = VectorLength(NiPoint3(initialPullObjRelativePosition.x, initialPullObjRelativePosition.y, 0)); // horizontal distance from hand to object
			float h = w * tanf(theta); // desired height relative to horizontal from hand
			//float delta_h = h - relObjPos.z;

			NiPoint3 horiz = VectorNormalized(NiPoint3(castDirection.x, castDirection.y, 0)) * w; // desired horizontal position relative to hand

			NiPoint3 deltaPos = NiPoint3(horiz.x, horiz.y, h) - relObjPos; // or + handPos - pullObj.pos instead of - relObjPos

			// Basic hand motions
			NiTransform inversePlayerTransform;
			hmdNode->m_worldTransform.Invert(inversePlayerTransform);

			NiPoint3 handPosLocal = inversePlayerTransform * handPos;

			NiPoint3 deltaHandPos = handPosLocal - prevHandPosLocal; // in hmd space

			NiPoint3 spellDirectionLocal = hmdNode->m_worldTransform.rot.Transpose() * castDirection; // transpose == inverse, since rotation matrix is orthogonal

			if (VectorLength(deltaHandPos) < 5.0f) { // Don't do anything if some weird jump happens
				float handSpeedInSpellDirection = DotProduct(deltaHandPos, spellDirectionLocal);
				if (handSpeedInSpellDirection < -1.0f) {
					pullDesired = true;
					_MESSAGE("Pull");
				}
				else if (handSpeedInSpellDirection > 1.0f) {
					pushDesired = true;
					_MESSAGE("Push");
				}
				else {
					pushDesired = false; // push should only be set in the frame that it happens (it's a 1-frame action)
				}
			}

			// Old force technique
			/*
			float weight = GetFormWeight(pullObj->baseForm);
			float negateGravityMagnitude = weight * 0.1012f; // experimentally determined constant
			NiPoint3 negateGravityForce(0, 0, negateGravityMagnitude);

			float coerceMagnitude = VectorLength(deltaPos) * 0.003f;
			coerceMagnitude = min(coerceMagnitude, 0.3f); // Cap force at some reasonable value
			NiPoint3 coerceForce = VectorNormalized(deltaPos) * coerceMagnitude;
			//NiPoint3 coerceForce(0, 0, coerceMagnitude);

			NiPoint3 force = negateGravityForce + coerceForce;

			float magnitude = VectorLength(force);
			NiPoint3 dir = force / magnitude;
			*/

			// New velocity technique
			auto collisionObj = (bhkCollisionObject *)(pullObj->loadedState->node->unk040);
			auto velocity = collisionObj->body->hkBody->motion.m_linearVelocity;

			auto hkPosition = collisionObj->body->hkBody->motion.m_motionState.m_transform.m_translation;
			float havokWorldScale = *HAVOK_WORLD_SCALE_ADDR;

			if (pullDesired) {
				float newMagnitude = VectorLength(relObjPos) * 0.1f;
				newMagnitude = min(newMagnitude, 10.0f); // Cap at some reasonable value
				NiPoint3 newVelocity = VectorNormalized(-relObjPos) * newMagnitude;

				ApplyHavokImpulse(vmRegistry, 0, pullObj, 0, 0, 1, 0); // 0 force, just to 'activate' it
				collisionObj->body->hkBody->motion.m_linearVelocity = { newVelocity.x, newVelocity.y, newVelocity.z, velocity.w };
			}
			else if (pushDesired) {
				NiPoint3 dir = VectorNormalized(relObjPos);
				float magnitude = 80.0f;
				ApplyHavokImpulse(vmRegistry, 0, pullObj, dir.x, dir.y, dir.z, magnitude);
				EffectShader_Stop(vmRegistry, 0, g_itemSelectedShader, pullObj);
				shaderApplied = false;
				pullObj = nullptr;
			}
			else {
				float newMagnitude = VectorLength(deltaPos) * 0.03f;
				newMagnitude = min(newMagnitude, 7.0f); // Cap at some reasonable value
				NiPoint3 newVelocity = VectorNormalized(deltaPos) * newMagnitude;

				ApplyHavokImpulse(vmRegistry, 0, pullObj, 0, 0, 1, 0); // 0 force, just to 'activate' it
				collisionObj->body->hkBody->motion.m_linearVelocity = { newVelocity.x, newVelocity.y, newVelocity.z, velocity.w };
			}
		}
	}


	prevPullObj = pullObj;
	prevHandPosLocal = handPosLocal;


	isLastUpdateValid = true;
}

extern "C" {	
	void OnDataLoaded()
	{
		const ModInfo *modInfo = DataHandler::GetSingleton()->LookupModByName("ForcePullVR.esp");
		if (!modInfo) {
			_MESSAGE("[CRITICAL] Could not get modinfo for this mod");
			return;
		}

		TESForm *shaderForm = LookupFormByID(GetFullFormID(modInfo, 0x3E90));
		if (shaderForm) {
			g_itemSelectedShader = DYNAMIC_CAST(shaderForm, TESForm, TESEffectShader);
			if (!g_itemSelectedShader) {
				_MESSAGE("Failed to cast selected item shader form");
				return;
			}
		}
		else {
			_MESSAGE("Failed to get slected item shader form");
			return;
		}

		UInt32 spellFormId = GetFullFormID(modInfo, 0xd65);
		TESForm *spellForm = LookupFormByID(spellFormId);
		if (spellForm) {
			g_debugSpell = DYNAMIC_CAST(spellForm, TESForm, SpellItem);
			if (!g_debugSpell) {
				_MESSAGE("Failed to cast spell form to spellitem");
				return;
			}
		}
		else {
			_MESSAGE("Failed to get spell form");
			return;
		}

		UInt32 sourceActivatorFormId = GetFullFormID(modInfo, 0x2906);
		UInt32 targetActivatorFormId = GetFullFormID(modInfo, 0x2907);
		TESForm *sourceActivatorForm = LookupFormByID(sourceActivatorFormId);
		TESForm *targetActivatorForm = LookupFormByID(targetActivatorFormId);
		if (sourceActivatorForm && targetActivatorForm) {
			debugSourceActivator = DYNAMIC_CAST(sourceActivatorForm, TESForm, TESObjectREFR);
			debugTargetActivator = DYNAMIC_CAST(targetActivatorForm, TESForm, TESObjectREFR);
			if (!debugSourceActivator || !debugTargetActivator) {
				_MESSAGE("Failed to cast source or target activators");
				return;
			}
		}
		else {
			_MESSAGE("Failed to get source or target activator forms");
			return;
		}
		_MESSAGE("Successfully loaded all forms");
	}

	void OnInputLoaded()
	{

	}

	// Listener for PapyrusVR Messages
	void OnPapyrusVRMessage(SKSEMessagingInterface::Message* msg)
	{
		if (msg) {
			if (msg->type == kPapyrusVR_Message_Init && msg->data) {
				_MESSAGE("PapyrusVR Init Message recived with valid data, registering for pose update callback");
				g_papyrusvr = (PapyrusVRAPI*)msg->data;
				g_papyrusvrManager = g_papyrusvr->GetVRManager();
				g_openvrHook = g_papyrusvr->GetOpenVRHook();
				if (!g_papyrusvrManager) {
					_MESSAGE("Could not get PapyrusVRManager");
					return;
				}
				if (!g_openvrHook) {
					_MESSAGE("Could not get OpenVRHook");
					return;
				}
				// Registers for PoseUpdates
				g_papyrusvrManager->RegisterVRUpdateListener(OnPoseUpdate);
			}
		}
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
				_MESSAGE("SKSE PostLoad recived, registering for SkyrimVRTools messages");
				g_messaging->RegisterListener(g_pluginHandle, "SkyrimVRTools", OnPapyrusVRMessage);
			}
			else if (msg->type == SKSEMessagingInterface::kMessage_PreLoadGame) {
				_MESSAGE("SKSE PreLoadGame message received");
				isLoaded = false;
			}
			else if (msg->type == SKSEMessagingInterface::kMessage_PostLoadGame || msg->type == SKSEMessagingInterface::kMessage_NewGame) {
				_MESSAGE("SKSE PostLoadGame or NewGame message received, type: %d", msg->type);
				isLoaded = true;
				vmRegistry = (*g_skyrimVM)->GetClassRegistry();
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

	bool ReadConfigOptions()
	{
		return true;
	}

	bool SKSEPlugin_Load(const SKSEInterface * skse)
	{	// Called by SKSE to load this plugin
		_MESSAGE("ForcePullVR loaded");

		// Registers for SKSE Messages (PapyrusVR probably still need to load, wait for SKSE message PostLoad)
		_MESSAGE("Registering for SKSE messages");
		g_messaging = (SKSEMessagingInterface*)skse->QueryInterface(kInterface_Messaging);
		g_messaging->RegisterListener(g_pluginHandle, "SKSE", OnSKSEMessage);

		if (ReadConfigOptions()) {
			_MESSAGE("Successfully read config parameters");
		}
		else {
			_WARNING("[WARNING] Failed to read config options. Using defaults instead.");
		}

		// wait for PapyrusVR init (during PostPostLoad SKSE Message)

		return true;
	}
};