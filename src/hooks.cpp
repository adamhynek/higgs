#include <mutex>

#include "xbyak/xbyak.h"
#include "skse64_common/Relocation.h"
#include "skse64_common/BranchTrampoline.h"
#include "skse64/NiGeometry.h"

#include "hooks.h"
#include "effects.h"
#include "RE/havok.h"
#include "grabber.h"
#include "vrikinterface001.h"

#include <Physics/Collide/Shape/Query/hkpShapeRayCastOutput.h>


auto shaderHookLoc = RelocAddr<uintptr_t>(0x2AE3E8);
auto shaderHookedFunc = RelocAddr<uintptr_t>(0x564DD0);

uintptr_t shaderHookedFuncAddr = 0;

auto worldUpdateHookLoc = RelocAddr<uintptr_t>(0x271EF9);

auto pickHookLoc = RelocAddr<uintptr_t>(0x6D2E7F);
auto pickHookedFunc = RelocAddr<uintptr_t>(0x3BA0C0); // CrosshairPickData::Pick

uintptr_t pickHookedFuncAddr = 0;

auto pickLinearCastHookLoc = RelocAddr<uintptr_t>(0x3BA3D7);
auto pickLinearCastHookedFunc = RelocAddr<uintptr_t>(0xAB5EC0); // ahkpWorld::LinearCast

uintptr_t pickLinearCastHookedFuncAddr = 0;

auto pickRayCastHookLoc = RelocAddr<uintptr_t>(0x3BA787);

auto shaderSetEffectDataHookLoc = RelocAddr<uintptr_t>(0x2292AE);
auto shaderSetEffectDataHookedFunc = RelocAddr<uintptr_t>(0x22A280); // BSLightingShaderProperty::SetEffectShaderData


ShaderReferenceEffect ** volatile g_shaderReferenceToSet = nullptr;
void HookedShaderReferenceEffectCtor(ShaderReferenceEffect *ref) // DO NOT USE THIS MEMORY YET - IT HAS JUST BEEN ALLOCATED. LET THE GAME CONSTRUCT IT IN A BIT.
{
	if (g_shaderReferenceToSet) {
		*g_shaderReferenceToSet = ref;
	}
}

typedef void(*_ShaderProperty_SetEffectData)(BSLightingShaderProperty *shaderProperty, void *effectShaderData);
void ShaderSetEffectDataHook(BSLightingShaderProperty *shaderProperty, void *effectShaderData, TESEffectShader *shader)
{
	if (shaderProperty->shaderFlags1 & BSShaderProperty::ShaderFlags1::kSLSF1_Facegen_Detail_Map ||
		shaderProperty->shaderFlags1 & BSShaderProperty::ShaderFlags1::kSLSF1_FaceGen_RGB_Tint) {
		if (shader == g_rightGrabber->itemSelectedShader || shader == g_rightGrabber->itemSelectedShaderOffLimits) {

			// It's skin geometry, skip this one
			return;
		}
	}

	((_ShaderProperty_SetEffectData)shaderSetEffectDataHookedFunc.GetUIntPtr())(shaderProperty, effectShaderData);
}

void WorldUpdateHook(bhkWorld *world)
{
	//_MESSAGE("Pre World update hook");
	// Perform the same operation both in this hook and in the main thread.
	// Why? We need it here to calm down physics constraints - they freak out if only set in the openvr hook
	// We also need to do it there though, since otherwise we get flickering lighting on the object.
	{
		std::lock_guard<std::mutex> lock(g_rightGrabber->deselectLock);

		if (g_rightGrabber->state == Grabber::State::Held) {
			PlayerCharacter *player = *g_thePlayer;
			NiAVObject *handNode = player->GetNiRootNode(1)->GetObjectByName(&g_rightGrabber->handNodeName.data);
			if (handNode) {
				NiPointer<TESObjectREFR> selectedObj;
				if (LookupREFRByHandle(g_rightGrabber->selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {
					NiAVObject *n = FindCollidableNode(g_rightGrabber->selectedObject.collidable);
					if (n) {
						NiTransform newTransform = handNode->m_worldTransform * g_rightGrabber->desiredObjTransformHandSpace;

						UpdateKeyframedNodeTransform(n, newTransform);
					}
				}
			}
		}
	}
	{
		std::lock_guard<std::mutex> lock(g_leftGrabber->deselectLock);

		if (g_leftGrabber->state == Grabber::State::Held) {
			PlayerCharacter *player = *g_thePlayer;
			NiAVObject *handNode = player->GetNiRootNode(1)->GetObjectByName(&g_leftGrabber->handNodeName.data);
			if (handNode) {
				NiPointer<TESObjectREFR> selectedObj;
				if (LookupREFRByHandle(g_leftGrabber->selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {
					NiAVObject *n = FindCollidableNode(g_leftGrabber->selectedObject.collidable);
					if (n) {
						NiTransform newTransform = handNode->m_worldTransform * g_leftGrabber->desiredObjTransformHandSpace;

						UpdateKeyframedNodeTransform(n, newTransform);
					}
				}
			}
		}
	}
}


void PickLinearCastHook(hkpWorld *world, const hkpCollidable* collA, const hkpLinearCastInput* input, hkpCdPointCollector* castCollector, hkpCdPointCollector* startCollector)
{
	bool displayLeft = g_leftGrabber->ShouldDisplayRollover();
	bool displayRight = g_rightGrabber->ShouldDisplayRollover();

	if (displayRight || displayLeft) {
		return;
	}

	((_hkpWorld_LinearCast)pickLinearCastHookedFuncAddr)(world, collA, input, castCollector, startCollector);
}


void PerformHooks(void)
{
	// First, set our addresses
	shaderHookedFuncAddr = shaderHookedFunc.GetUIntPtr();
	pickHookedFuncAddr = pickHookedFunc.GetUIntPtr();
	pickLinearCastHookedFuncAddr = pickLinearCastHookedFunc.GetUIntPtr();

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
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
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(shaderHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("Shader hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
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
				mov(rax, (uintptr_t)WorldUpdateHook);
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
				dq(worldUpdateHookLoc.GetUIntPtr() + 7);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(worldUpdateHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("World update hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
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
				mov(rax, (uintptr_t)PickLinearCastHook);
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
}
