#include <mutex>
#include <regex>

#include "xbyak/xbyak.h"
#include "skse64_common/Relocation.h"
#include "skse64_common/BranchTrampoline.h"
#include "skse64/NiGeometry.h"
#include "skse64_common/SafeWrite.h"
#include "skse64/Hooks_UI.h"
#include "skse64/GameRTTI.h"

#include "hooks.h"
#include "effects.h"
#include "RE/havok.h"
#include "RE/offsets.h"
#include "hand.h"
#include "config.h"
#include "main.h"
#include "finger_curves.h"
#include "menu_checker.h"
#include "pluginapi.h"

#include <Physics/Collide/Shape/Query/hkpShapeRayCastOutput.h>


uintptr_t shaderHookedFuncAddr = 0;
auto shaderHookLoc = RelocAddr<uintptr_t>(0x2AE3E8);
auto shaderHookedFunc = RelocAddr<uintptr_t>(0x564DD0);

uintptr_t pickHookedFuncAddr = 0;
auto pickHookLoc = RelocAddr<uintptr_t>(0x6D2E7F);
auto pickHookedFunc = RelocAddr<uintptr_t>(0x3BA0C0); // CrosshairPickData::Pick

uintptr_t pickLinearCastHookedFuncAddr = 0;
auto pickLinearCastHookLoc = RelocAddr<uintptr_t>(0x3BA3D7);
auto pickLinearCastHookedFunc = RelocAddr<uintptr_t>(0xAB5EC0); // ahkpWorld::LinearCast

uintptr_t postWandUpdateHookedFuncAddr = 0;
auto postWandUpdateHookLoc = RelocAddr<uintptr_t>(0x13233AA); // A call shortly after the wand nodes are updated as part of Main::Draw()
auto postWandUpdateHookedFunc = RelocAddr<uintptr_t>(0xDCF900);

uintptr_t preVRIKMainThreadHookedFuncAddr = 0;
auto preVRIKMainThreadHookLoc = RelocAddr<uintptr_t>(0x5BABB5); // A call on the main thread right after the skse process events hook, and right before the vrik main thread hook
auto preVRIKMainThreadHookedFunc = RelocAddr<uintptr_t>(0xDFE470); // Some bhkWorld func

uintptr_t preVRIKPlayerCharacterUpdateHookedFuncAddr = 0;
auto preVRIKPlayerCharacterUpdateHookLoc = RelocAddr<uintptr_t>(0x6ABCBF); // A call in PlayerCharacter::Update after AlignClaviclesToHand, and right before the vrik hook in there
auto preVRIKPlayerCharacterUpdateHookedFunc = RelocAddr<uintptr_t>(0x6AA060); // PlayerCharacter::UpdateVRBlocking

uintptr_t postVRIKPlayerCharacterUpdateHookedFuncAddr = 0;
auto postVRIKPlayerCharacterUpdateHookLoc = RelocAddr<uintptr_t>(0x6ABCFC); // A call in PlayerCharacter::Update after AlignClaviclesToHand, and right before the vrik hook in there
auto postVRIKPlayerCharacterUpdateHookedFunc = RelocAddr<uintptr_t>(0x62ED20); // Actor::GetWeapon

uintptr_t playerCharacterUpdateHookedFuncAddr = 0;
auto playerCharacterUpdateHookLoc = RelocAddr<uintptr_t>(0x649FD3); // In Job_Non_render_safe_AI(), calls PlayerCharacter::Update()
auto playerCharacterHookedFunc = RelocAddr<uintptr_t>(0x6C6910); // PlayerCharacter::Update()

uintptr_t updateVRMeleeDataRigidBodyCtorHookedFuncAddr = 0;
auto updateVRMeleeDataRigidBodyCtorHookLoc = RelocAddr<uintptr_t>(0x6B0843); // In PlayerCharacter::UpdateVRMeleeData()
auto updateVRMeleeDataRigidBodyCtorHookedFunc = RelocAddr<uintptr_t>(0x2AEC80); // bhkRigidBody_ctor()

auto hideSpellOriginLoc = RelocAddr<uintptr_t>(0x6AC012); // write 7 bytes of nops here

auto startGrabObjectLoc = RelocAddr<uintptr_t>(0x6CC000);

auto allocMeleeRigidBodySizeLoc = RelocAddr<uintptr_t>(0x6B0828);

uintptr_t updatePhysicsTimesHookedFuncAddr = 0;
auto updatePhysicsTimesHookLoc = RelocAddr<uintptr_t>(0x5BBAEF);
auto updatePhysicsTimesHookedFunc = RelocAddr<uintptr_t>(0xDFB3C0);

auto bhkCollisionFilter_CompareFilterInfo_HookLoc = RelocAddr<uintptr_t>(0xE2BA10);

auto getActivateTextHookLoc = RelocAddr<uintptr_t>(0x6D3337);

auto refreshActivateButtonArtHookLoc = RelocAddr<uintptr_t>(0x53F18E);

auto pickRayCastHookLoc = RelocAddr<uintptr_t>(0x3BA787);

auto worldUpdateHookLoc = RelocAddr<uintptr_t>(0x271EF9);

auto physicsUpdateHookLoc = RelocAddr<uintptr_t>(0xDFB554);

auto shaderSetEffectDataHookLoc = RelocAddr<uintptr_t>(0x2292AE);
auto shaderSetEffectDataHookedFunc = RelocAddr<uintptr_t>(0x22A280); // BSLightingShaderProperty::SetEffectShaderData

uintptr_t shaderSetEffectDataInitHookedFuncAddr = 0;
auto shaderSetEffectDataInitHookLoc = RelocAddr<uintptr_t>(0x56761E);
auto shaderSetEffectDataInitHookedFunc = RelocAddr<uintptr_t>(0x2291F0); // TESEffectShader::SetEffectShaderData

auto shaderCheckFlagsHookLoc = RelocAddr<uintptr_t>(0x229243);

auto prePhysicsStepHookLoc = RelocAddr<uintptr_t>(0xDFB709);

auto GetRefFromCollidable_GetNodeFromCollidable_HookLoc = RelocPtr<_GetNodeFromCollidable>(0x3B495B);

auto TriggerEntry_RegisterOverLap_Actor_IsInRagdollState_HookLoc = RelocPtr<_GetNodeFromCollidable>(0x3B6E23);


bool TriggerEntry_RegisterOverLap_Actor_IsInRagdollState_Hook(Actor *actor)
{
	// While an actor is not ragdolled, a trigger will only work for it with objects on the character controller layer, so we make the trigger think the player is ragdolled.
	// This IsInRagdollState check is only used for that purpose by the trigger.

	if (actor == (*g_thePlayer)) {
		return true;
	}
	return Actor_IsInRagdollState(actor);
}


NiAVObject * GetRefFromCollidable_GetNodeFromCollidable_Hook(hkpCollidable *collidable)
{
	// When the game wants to know which refr a collidable belongs do, we want our hands to be treated as if they belong to the player.
	// The PlayerWorldNode is treated as belonging to the player, so we pretend the hands belong to the player world node.

	if (collidable && (collidable == g_rightHand->handCollidable || collidable == g_leftHand->handCollidable)) {
		return (*g_thePlayer)->unk3F0[PlayerCharacter::Node::kNode_PlayerWorldNode]; // even if it's null, that's fine
	}

	return GetNodeFromCollidable(collidable);
}


ShaderReferenceEffect ** volatile g_shaderReferenceToSet = nullptr;
void HookedShaderReferenceEffectCtor(ShaderReferenceEffect *ref) // DO NOT USE THIS MEMORY YET - IT HAS JUST BEEN ALLOCATED. LET THE GAME CONSTRUCT IT IN A BIT.
{
	if (g_shaderReferenceToSet) {
		*g_shaderReferenceToSet = ref;
	}
}

BSGeometry *g_shaderGeometry = nullptr; // This gets set shortly before the below hook gets called
ShaderReferenceEffect *g_shaderReference = nullptr; // This gets set before the SetEffectData calls

typedef void(*_ShaderProperty_SetEffectData)(BSLightingShaderProperty *shaderProperty, void *effectShaderData);
void ShaderSetEffectDataHook(BSLightingShaderProperty *shaderProperty, void *effectShaderData, TESEffectShader *shader)
{
	if (shader == g_rightHand->itemSelectedShader || shader == g_rightHand->itemSelectedShaderOffLimits) {
		{
			if (!g_shaderReference) return;

			std::shared_lock lock(g_shaderNodesLock);

			// We only play the shader on geometry that's in the set
			if (g_shaderNodes.count(g_shaderReference) == 0) return;
			if (g_shaderNodes[g_shaderReference].count(g_shaderGeometry) == 0) return;
		}
	}

	((_ShaderProperty_SetEffectData)shaderSetEffectDataHookedFunc.GetUIntPtr())(shaderProperty, effectShaderData);
}

UInt64 g_pickValue = 0; // This gets set shortly before the below hook gets called
UInt32 g_pickedHandle = 0;

void PickLinearCastHook(hkpWorld *world, const hkpCollidable* collA, const hkpLinearCastInput* input, hkpCdPointCollector* castCollector, hkpCdPointCollector* startCollector)
{
	Hand *rolloverHand = GetHandToShowRolloverFor();

	bool isLeftHanded = *g_leftHandedMode;

	if (rolloverHand) {
		SetSelectedHandles(isLeftHanded, rolloverHand->selectedObject.handle);
		g_pickedHandle = *g_invalidRefHandle;
		return;
	}

	((_hkpWorld_LinearCast)pickLinearCastHookedFuncAddr)(world, collA, input, castCollector, startCollector);

	if (g_pickValue == 2) { // The pick linear cast gets called multiple times. The selected handles are only set when this value is 2 /shrug
		CrosshairPickData *pickData = *g_pickData;
		if (pickData) {
			bool isLeftHanded = *g_leftHandedMode;
			g_pickedHandle = isLeftHanded ? pickData->leftHandle1 : pickData->rightHandle1;
		}
	}
}


struct RefreshActivateButtonArtTask : UIDelegate_v1
{
	virtual void Run() {
		if (g_wsActivateRollover) {
			// Unstable if called from random places
			RefreshActivateButtonArt(g_wsActivateRollover);
		}
	}
	virtual void Dispose() {
		delete this;
	}
};

bool wasRolloverSet = false;
bool hasSavedRollover = false;
NiTransform normalRolloverTransform;
bool hasSavedRumbleIntensity = false;
float normalRumbleIntensity;
double lastRolloverSetTime = 0;

void PostWandUpdateHook()
{
	PlayerCharacter *player = *g_thePlayer;

	g_rightHand->LateMainThreadUpdate();
	g_leftHand->LateMainThreadUpdate();

	static BSFixedString rolloverNodeStr("WSActivateRollover");
	NiPointer<NiAVObject> roomNode = player->unk3F0[PlayerCharacter::Node::kNode_RoomNode];
	NiPointer<NiAVObject> rolloverNode = roomNode ? roomNode->GetObjectByName(&rolloverNodeStr.data) : nullptr;

	// This hook is on the main thread, so we're kind of okay to do stuff with the Hand as this won't interleave with its Update

	Hand *rolloverHand = GetHandToShowRolloverFor();

	if (rolloverHand) {
		// Something is grabbed

		if (!hasSavedRollover) {
			if (rolloverNode) {
				normalRolloverTransform = rolloverNode->m_localTransform;
				hasSavedRollover = true;
			}
		}

		rolloverHand->SetupRollover(rolloverNode);

		Setting	* activateRumbleIntensitySetting = GetINISetting("fActivateRumbleIntensity:VRInput");
		if (!hasSavedRumbleIntensity) {
			hasSavedRumbleIntensity = true;
			normalRumbleIntensity = activateRumbleIntensitySetting->data.f32;
		}
		activateRumbleIntensitySetting->SetDouble(0);

		if (Config::options.overrideActivateText) {
			g_overrideActivateText = rolloverHand->GetActivateText(g_overrideActivateTextStr);

			bool overrideActivateButtonBefore = g_overrideActivateButton;
			std::string activateButtonStrBefore = g_overrideActivateButtonStr;
			g_overrideActivateButton = rolloverHand->GetActivateButton(g_overrideActivateButtonStr);

			if (g_overrideActivateButton != overrideActivateButtonBefore || (g_overrideActivateButton && g_overrideActivateButtonStr != activateButtonStrBefore)) {
				// Just starting/ending to override, or we're still overriding but the button changed
				g_taskInterface->AddUITask(new RefreshActivateButtonArtTask());
			}
		}
	}
	else {
		if (wasRolloverSet) {
			// Nothing is grabbed, and something was last time

			if (hasSavedRumbleIntensity) {
				Setting * activateRumbleIntensitySetting = GetINISetting("fActivateRumbleIntensity:VRInput");
				activateRumbleIntensitySetting->data.f32 = normalRumbleIntensity;
			}

			SetGeometryAlphaDownstream(rolloverNode, 1.0f); // Restore the alpha to 1 in case it was modified when showing on the hand

			lastRolloverSetTime = g_currentFrameTime;
			rolloverNode->m_localTransform.scale = 0.000001f; // Hide the rollover for a bit after letting go of something

			NiAVObject::ControllerUpdateContext ctx{ 0, 0 };
			NiAVObject_UpdateNode(rolloverNode, &ctx);

			g_overrideActivateText = false;
			g_overrideActivateButton = false;
			g_taskInterface->AddUITask(new RefreshActivateButtonArtTask());
		}

		if (g_currentFrameTime - lastRolloverSetTime > Config::options.rolloverHideTime) {
			if (rolloverNode && hasSavedRollover) {
				rolloverNode->m_localTransform = normalRolloverTransform;
			}
		}
	}

	wasRolloverSet = rolloverHand != nullptr;


	if (!Config::options.disableSelectionBeam) {
		NiAVObject *spellOrigin = player->unk3F0[PlayerCharacter::Node::kNode_SpellOrigin];
		NiNode *spellOriginNode = spellOrigin ? spellOrigin->GetAsNiNode() : nullptr;
		if (spellOriginNode && spellOriginNode->m_children.m_emptyRunStart >= 2) {
			if (g_rightHand->state == Hand::State::SelectionLocked) {
				g_rightHand->SetupSelectionBeam(spellOriginNode);
			}
			else if (g_leftHand->state == Hand::State::SelectionLocked) {
				g_leftHand->SetupSelectionBeam(spellOriginNode);
			}
			else {
				spellOriginNode->m_flags |= 1; // hide spell origin
			}
		}
	}

	if (!g_isVrikPresent) {
		if (NiPointer<NiAVObject> offhandNode = player->unk3F0[*g_leftHandedMode ? PlayerCharacter::Node::kNode_RightControllerNode : PlayerCharacter::Node::kNode_LeftControllerNode]) {
			if (MenuChecker::isGameStopped()) { // A menu is open
				offhandNode->m_localTransform.scale = 1.0f;
			}
			else {
				TESObjectWEAP *mainHandEquippedWeapon = GetEquippedWeapon(player, false);
				if (mainHandEquippedWeapon && IsTwoHanded(mainHandEquippedWeapon)) {
					offhandNode->m_localTransform.scale = 0.0001f; // effectively hide the offhand controller node
				}
			}
		}
	}
}


void PlayerCharacterUpdateHook()
{
	HiggsPluginAPI::TriggerPreVrikPreHiggsCallbacks();
	Update();
	HiggsPluginAPI::TriggerPreVrikPostHiggsCallbacks();
}


void PostVRIKPCUpdateHook()
{
	HiggsPluginAPI::TriggerPostVrikPreHiggsCallbacks();

	g_rightHand->PostVrikUpdate();
	g_leftHand->PostVrikUpdate();

	if (Config::options.dontAnimateFingersWhenBeast && TESRace_IsBeast(Actor_GetRace(*g_thePlayer))) {
		g_rightHand->fingerAnimator.animate = false;
		g_leftHand->fingerAnimator.animate = false;
	}
	else {
		g_rightHand->fingerAnimator.Update();
		g_leftHand->fingerAnimator.Update();
	}

	HiggsPluginAPI::TriggerPostVrikPostHiggsCallbacks();
}


BSString *g_activateText = nullptr;
bool g_overrideActivateText = false;
std::string g_overrideActivateTextStr;

void GetActivateTextHook()
{
	if (!g_overrideActivateText) {
		return;
	}

	BSString *str = g_activateText;
	if (str) {
		char *text = str->m_data;
		if (text && *text) {
			ReplaceBSString(*str, g_overrideActivateTextStr);
		}
	}
}


BSFixedString *g_activateButtonName = nullptr;
const char **g_activateButtonNameArg = nullptr;
void *g_wsActivateRollover = nullptr;
bool g_overrideActivateButton = false;
std::string g_overrideActivateButtonStr;

void RefreshActivateButtonArtHook()
{
	BSFixedString *buttonName = g_activateButtonName;
	if (buttonName) {
		if (g_overrideActivateButton) {
			BSFixedString newButtonName(g_overrideActivateButtonStr.c_str());
			BSFixedString_Copy(buttonName, &newButtonName);
			newButtonName.Release();
			*g_activateButtonNameArg = buttonName->data;
		}
		else if (g_isActivateBoundToGrip) {
			BSFixedString newButtonName("grip");
			BSFixedString_Copy(buttonName, &newButtonName);
			newButtonName.Release();
			*g_activateButtonNameArg = buttonName->data;
		}
	}
}


void UpdatePhysicsTimesHook()
{
	float deltaTime = min(*g_secondsSinceLastFrame_Unmultiplied, Config::options.havokMaxMaxTime);
	*fMaxTime = deltaTime;
	*fMaxTimeComplex = deltaTime * Config::options.havokMaxTimeComplexMultiplier;
}


auto bhkRigidBodyT_vtbl = RelocAddr<void *>(0x182BA80);
void UpdateVRMeleeDataRigidBodyCtorHook(bhkRigidBody *newRigidBody, NiAVObject *root)
{
	NiPointer<bhkRigidBody> rigidBody = GetRigidBody(root);
	bhkRigidBodyT *rigidBodyT = DYNAMIC_CAST(rigidBody, bhkRigidBody, bhkRigidBodyT);
	if (rigidBodyT) {
		*((void **)newRigidBody) = ((void *)(bhkRigidBodyT_vtbl)); // set vtbl
		bhkRigidBodyT *newRigidBodyT = DYNAMIC_CAST(newRigidBody, bhkRigidBody, bhkRigidBodyT);
		newRigidBodyT->rotation = rigidBodyT->rotation;
		newRigidBodyT->translation = rigidBodyT->translation;
	}
}

using CollisionFilterComparisonResult = HiggsPluginAPI::IHiggsInterface001::CollisionFilterComparisonResult;
CollisionFilterComparisonResult bhkCollisionFilter_CompareFilterInfo_Hook(bhkCollisionFilter *filter, UInt32 filterInfoA, UInt32 filterInfoB)
{
	return HiggsPluginAPI::TriggerCollisionFilterComparisonCallbacks(filter, filterInfoA, filterInfoB);
}

void PrePhysicsStepHook(bhkWorld *world)
{
	HiggsPluginAPI::TriggerPrePhysicsStepCallbacks(world);
}

typedef void(*_Actor_Update)(Actor *_this, float a_delta);
_Actor_Update g_originalPCUpdate = nullptr;
static RelocPtr<_Actor_Update> PlayerCharacter_Update_vtbl(0x16E27A8); // 0x16E2230 + 0xAF * 8
void PlayerCharacter_Update_Hook(PlayerCharacter *_this, float delta)
{
	g_originalPCUpdate(_this, delta);

	if (Config::options.debugDrawControllers) {
		if (NiPointer<NiAVObject> rightController = _this->unk3F0[PlayerCharacter::Node::kNode_RightControllerNode]) {
			rightController->m_localTransform.pos = { 0.f, 0.f, 0.f };
			rightController->m_localTransform.scale = 1.0f;
			rightController->m_flags &= ~1;

			NiAVObject::ControllerUpdateContext ctx{ 0, 0 };
			NiAVObject_UpdateNode(rightController, &ctx);
		}
		if (NiPointer<NiAVObject> leftController = _this->unk3F0[PlayerCharacter::Node::kNode_LeftControllerNode]) {
			leftController->m_localTransform.pos = { 0.f, 0.f, 0.f };
			leftController->m_localTransform.scale = 1.0f;
			leftController->m_flags &= ~1;

			NiAVObject::ControllerUpdateContext ctx{ 0, 0 };
			NiAVObject_UpdateNode(leftController, &ctx);
		}
	}
}


void PerformHooks(void)
{
	// First, set our addresses
	shaderHookedFuncAddr = shaderHookedFunc.GetUIntPtr();
	pickHookedFuncAddr = pickHookedFunc.GetUIntPtr();
	pickLinearCastHookedFuncAddr = pickLinearCastHookedFunc.GetUIntPtr();
	shaderSetEffectDataInitHookedFuncAddr = shaderSetEffectDataInitHookedFunc.GetUIntPtr();
	postWandUpdateHookedFuncAddr = postWandUpdateHookedFunc.GetUIntPtr();
	preVRIKMainThreadHookedFuncAddr = preVRIKMainThreadHookedFunc.GetUIntPtr();
	preVRIKPlayerCharacterUpdateHookedFuncAddr = preVRIKPlayerCharacterUpdateHookedFunc.GetUIntPtr();
	postVRIKPlayerCharacterUpdateHookedFuncAddr = postVRIKPlayerCharacterUpdateHookedFunc.GetUIntPtr();
	playerCharacterUpdateHookedFuncAddr = playerCharacterHookedFunc.GetUIntPtr();
	updatePhysicsTimesHookedFuncAddr = updatePhysicsTimesHookedFunc.GetUIntPtr();
	updateVRMeleeDataRigidBodyCtorHookedFuncAddr = updateVRMeleeDataRigidBodyCtorHookedFunc.GetUIntPtr();

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				push(rcx);
				push(rdx);
				sub(rsp, 0x20); // Need an additional 0x20 bytes for scratch space

				// Call our hook
				mov(rax, (uintptr_t)HookedShaderReferenceEffectCtor);
				call(rax);

				add(rsp, 0x20);
				pop(rdx);
				pop(rcx);

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
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(shaderHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("ShaderReferenceEffect ctor hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				// Save r14, as it has a value that determines if we should read the selected handles or not
				mov(rax, (uintptr_t)&g_pickValue);
				mov(ptr[rax], r14);

				// Call our hook
				mov(rax, (uintptr_t)PickLinearCastHook);
				call(rax);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(pickLinearCastHookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(pickLinearCastHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("Pick linear cast hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				// Call our hook
				mov(r8, ptr[rsp + 0x40]); // TESEffectShader passed into the parent function is at rsp + 0x40
				mov(rax, (uintptr_t)ShaderSetEffectDataHook);
				call(rax);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(shaderSetEffectDataHookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(shaderSetEffectDataHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("Shader SetEffectShaderData hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				// The node the shader is considering playing on is at rdx at this point.
				// During this hooked function call, rdx gets overwritten so we need to grab it here.
				push(rax);
				mov(rax, (uintptr_t)&g_shaderGeometry);
				mov(ptr[rax], rdx);
				pop(rax);

				// Original code
				call(ptr[rax + 0x1C8]);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(shaderCheckFlagsHookLoc.GetUIntPtr() + 6);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(shaderCheckFlagsHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("Shader check flags hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				// The ShaderReferenceEffect is at rsi at this point.
				push(rax);
				mov(rax, (uintptr_t)&g_shaderReference);
				mov(ptr[rax], rsi);
				pop(rax);

				// Original code
				mov(rax, shaderSetEffectDataInitHookedFuncAddr);
				call(rax);

				// Set g_shaderReference to null so that other calls to SetEffectData (if there are any...) don't use it
				// We're cool to use rcx and rdx at this point, because they're caller-save and could have been overwritten with anything.
				mov(rcx, (uintptr_t)&g_shaderReference);
				mov(rdx, 0);
				mov(ptr[rcx], rdx);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(shaderSetEffectDataInitHookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(shaderSetEffectDataInitHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("Shader SetEffectShaderDataInit hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				// Original code
				call(ptr[rax + 0x260]);

				mov(rdx, rbp);
				sub(rdx, 0x28);
				mov(rcx, (uintptr_t)&g_activateText);
				mov(ptr[rcx], rdx);

				push(rax);
				sub(rsp, 0x38); // Need to keep the stack 16 byte aligned, and an additional 0x20 bytes for scratch space
				movsd(ptr[rsp + 0x20], xmm0);

				// Call our hook
				mov(rax, (uintptr_t)GetActivateTextHook);
				call(rax);

				movsd(xmm0, ptr[rsp + 0x20]);
				add(rsp, 0x38);
				pop(rax);

				// TODO: GetActivateText returns a bool, if that bool is false the rollover menu is not set

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(getActivateTextHookLoc.GetUIntPtr() + 6);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(getActivateTextHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("GetActivateText hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				// rbp + 0x20 has the current BSFixedString
				// rbp - 0x08 has the actual arg the refresh function uses, which just points to the char* owned by the BSFixedString

				push(rax);
				push(rcx);
				push(rdx);
				push(r8);
				push(r9);
				sub(rsp, 0x28); // Need an additional 0x20 bytes for scratch space and align the stack

				mov(rcx, rbp);
				add(rcx, 0x20);
				mov(rax, (uintptr_t)&g_activateButtonName);
				mov(ptr[rax], rcx);
				mov(rcx, rbp);
				sub(rcx, 0x8);
				mov(rax, (uintptr_t)&g_activateButtonNameArg);
				mov(ptr[rax], rcx);
				mov(rax, (uintptr_t)&g_wsActivateRollover);
				mov(ptr[rax], rdi);

				// Call our hook
				mov(rax, (uintptr_t)RefreshActivateButtonArtHook);
				call(rax);

				add(rsp, 0x28);
				pop(r9);
				pop(r8);
				pop(rdx);
				pop(rcx);
				pop(rax);

				// Original code
				call(ptr[rax + 0xB8]);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(refreshActivateButtonArtHookLoc.GetUIntPtr() + 6);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(refreshActivateButtonArtHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("RefreshActivateButtonArt hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				// Original code
				mov(rax, postWandUpdateHookedFuncAddr);
				call(rax);

				push(rax);
				sub(rsp, 0x38); // Need to keep the stack 16 byte aligned, and an additional 0x20 bytes for scratch space
				movsd(ptr[rsp + 0x20], xmm0);

				// Call our hook
				mov(rax, (uintptr_t)PostWandUpdateHook);
				call(rax);

				movsd(xmm0, ptr[rsp + 0x20]);
				add(rsp, 0x38);
				pop(rax);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(postWandUpdateHookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(postWandUpdateHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("Post Wand Update hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				push(rcx);
				sub(rsp, 0x28); // Need to keep the stack 16 byte aligned, and an additional 0x20 bytes for scratch space

				// Call our hook
				mov(rax, (uintptr_t)PlayerCharacterUpdateHook);
				call(rax);

				add(rsp, 0x28);
				pop(rcx);

				// Original code
				mov(rax, preVRIKPlayerCharacterUpdateHookedFuncAddr);
				call(rax);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(preVRIKPlayerCharacterUpdateHookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(preVRIKPlayerCharacterUpdateHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("PlayerCharacter::Update pre-vrik hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				// Original code
				mov(rax, postVRIKPlayerCharacterUpdateHookedFuncAddr);
				call(rax);

				push(rax);
				sub(rsp, 0x38); // Need to keep the stack 16 byte aligned, and an additional 0x20 bytes for scratch space
				movsd(ptr[rsp + 0x20], xmm0);

				// Call our hook
				mov(rax, (uintptr_t)PostVRIKPCUpdateHook);
				call(rax);

				movsd(xmm0, ptr[rsp + 0x20]);
				add(rsp, 0x38);
				pop(rax);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(postVRIKPlayerCharacterUpdateHookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(postVRIKPlayerCharacterUpdateHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("PlayerCharacter::Update post-vrik hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				// Original code
				mov(rax, updateVRMeleeDataRigidBodyCtorHookedFuncAddr);
				call(rax);

				push(rax);
				sub(rsp, 0x38); // Need to keep the stack 16 byte aligned, and an additional 0x20 bytes for scratch space
				movsd(ptr[rsp + 0x20], xmm0);

				mov(rcx, rax);
				mov(rdx, ptr[rsp + 0x70]); // root node of cloned model

				// Call our hook
				mov(rax, (uintptr_t)UpdateVRMeleeDataRigidBodyCtorHook);
				call(rax);

				movsd(xmm0, ptr[rsp + 0x20]);
				add(rsp, 0x38);
				pop(rax);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(updateVRMeleeDataRigidBodyCtorHookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(updateVRMeleeDataRigidBodyCtorHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("PlayerCharacter::UpdateVRMeleeData bhkRigidBody_ctor hook complete");
	}

	if (Config::options.enableHavokFix) {
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				push(rdx);
				sub(rsp, 0x8); // Need to keep the stack SIXTEEN BYTE ALIGNED
				movsd(ptr[rsp], xmm0);

				// Call our hook
				mov(rax, (uintptr_t)UpdatePhysicsTimesHook);
				call(rax);

				movsd(xmm0, ptr[rsp]);
				add(rsp, 0x8);
				pop(rdx);

				// Original code
				mov(rax, updatePhysicsTimesHookedFuncAddr);
				call(rax);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(updatePhysicsTimesHookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(updatePhysicsTimesHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("Update Physics Times hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack, ret0, ret1;

				push(rcx);
				push(rdx);
				push(r8);
				sub(rsp, 0x28); // Need to keep the stack 16 byte aligned, and an additional 0x20 bytes for scratch space

				// Call our hook
				mov(rax, (uintptr_t)bhkCollisionFilter_CompareFilterInfo_Hook);
				call(rax);

				add(rsp, 0x28);
				pop(r8);
				pop(rdx);
				pop(rcx);

				// Original code
				mov(r10d, edx);
				mov(r11, rcx);

				cmp(al, UInt8(CollisionFilterComparisonResult::Collide));
				je(ret1); // force collision

				cmp(al, UInt8(CollisionFilterComparisonResult::Ignore));
				je(ret0); // ignore collision

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(ret0);
				xor(al, al);
				ret();

				L(ret1);
				mov(al, 1);
				ret();

				L(jumpBack);
				dq(bhkCollisionFilter_CompareFilterInfo_HookLoc.GetUIntPtr() + 6);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write6Branch(bhkCollisionFilter_CompareFilterInfo_HookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("bhkCollisionFilter::CompareFilterInfo hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				sub(rsp, 0x20); // Need an additional 0x20 bytes for scratch space

				mov(rcx, r12); // the bhkWorld is at r12

				// Call our hook
				mov(rax, (uintptr_t)PrePhysicsStepHook);
				call(rax);

				add(rsp, 0x20);

				// Original code
				mov(rcx, r13);
				test(r14b, r14b);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(prePhysicsStepHookLoc.GetUIntPtr() + 6);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write6Branch(prePhysicsStepHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("Pre-physics-step hook complete");
	}

	if (!Config::options.disableSelectionBeam) {
		UInt64 nops = 0x9090909090909090;
		SafeWriteBuf(hideSpellOriginLoc.GetUIntPtr(), &nops, 7);
		_MESSAGE("NOP'd out SpellOrigin hide");
	}

	if (Config::options.disableVanillaGrab) {
		UInt8 ret = 0xC3;
		SafeWrite8(startGrabObjectLoc.GetUIntPtr(), ret);
		_MESSAGE("ret'd out PlayerCharacter::StartGrabObject");
	}

	if (Config::options.treatHandCollisionAsBelongingToPlayer) {
		g_branchTrampoline.Write5Call(GetRefFromCollidable_GetNodeFromCollidable_HookLoc.GetUIntPtr(), uintptr_t(GetRefFromCollidable_GetNodeFromCollidable_Hook));
		_MESSAGE("GetRefFromCollidable GetNodeFromCollidable hook complete");
	}

	if (Config::options.allowAllPlayerCollisionForTriggers) {
		g_branchTrampoline.Write5Call(TriggerEntry_RegisterOverLap_Actor_IsInRagdollState_HookLoc.GetUIntPtr(), uintptr_t(TriggerEntry_RegisterOverLap_Actor_IsInRagdollState_Hook));
		_MESSAGE("TriggerEntry::RegisterOverLap Actor::IsInRagdollState hook complete");
	}

	{
		UInt64 bytes = 0x00000060B9; // mov ecx, 0x60
		SafeWriteBuf(allocMeleeRigidBodySizeLoc.GetUIntPtr(), &bytes, 5);
		_MESSAGE("Patched allocation for melee rigidbody to allocate 0x60 bytes instead of 0x40");
	}

	{
		g_originalPCUpdate = *PlayerCharacter_Update_vtbl;
		SafeWrite64(PlayerCharacter_Update_vtbl.GetUIntPtr(), uintptr_t(PlayerCharacter_Update_Hook));
	}
}
