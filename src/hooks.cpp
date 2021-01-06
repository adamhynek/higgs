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

uintptr_t shaderSetEffectDataInitHookedFuncAddr = 0;
auto shaderSetEffectDataInitHookLoc = RelocAddr<uintptr_t>(0x56761E);
auto shaderSetEffectDataInitHookedFunc = RelocAddr<uintptr_t>(0x2291F0); // TESEffectShader::SetEffectShaderData

auto shaderCheckFlagsHookLoc = RelocAddr<uintptr_t>(0x229243);


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
	if (shader == g_rightGrabber->itemSelectedShader || shader == g_rightGrabber->itemSelectedShaderOffLimits) {
		{
			if (!g_shaderReference) return;

			std::shared_lock lock(g_shaderNodesLock);

			// We only play the shader on geometry that's in the set
			auto &shaderNodes = *g_shaderNodes;
			if (shaderNodes.count(g_shaderReference) == 0) return;
			if (shaderNodes[g_shaderReference].count(g_shaderGeometry) == 0) return;
		}
	}

	((_ShaderProperty_SetEffectData)shaderSetEffectDataHookedFunc.GetUIntPtr())(shaderProperty, effectShaderData);
}

void WorldUpdateHook(bhkWorld *world)
{/*
	//_MESSAGE("Pre World update hook");
	// Perform the same operation both in this hook and in the main thread.
	// Why? We need it here to calm down physics constraints - they freak out if only set in the openvr hook
	// We also need to do it there though, since otherwise we get flickering lighting on the object.
	{
		std::scoped_lock lock(g_rightGrabber->deselectLock);

		if (g_rightGrabber->state == Grabber::State::Held) {
			PlayerCharacter *player = *g_thePlayer;
			NiAVObject *handNode = player->GetNiRootNode(1)->GetObjectByName(&g_rightGrabber->handNodeName.data);
			if (handNode) {
				NiPointer<TESObjectREFR> selectedObj;
				if (LookupREFRByHandle(g_rightGrabber->selectedObject.handle, selectedObj) && selectedObj->GetNiNode()) {
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
		std::scoped_lock lock(g_leftGrabber->deselectLock);

		if (g_leftGrabber->state == Grabber::State::Held) {
			PlayerCharacter *player = *g_thePlayer;
			NiAVObject *handNode = player->GetNiRootNode(1)->GetObjectByName(&g_leftGrabber->handNodeName.data);
			if (handNode) {
				NiPointer<TESObjectREFR> selectedObj;
				if (LookupREFRByHandle(g_leftGrabber->selectedObject.handle, selectedObj) && selectedObj->GetNiNode()) {
					NiAVObject *n = FindCollidableNode(g_leftGrabber->selectedObject.collidable);
					if (n) {
						NiTransform newTransform = handNode->m_worldTransform * g_leftGrabber->desiredObjTransformHandSpace;

						UpdateKeyframedNodeTransform(n, newTransform);
					}
				}
			}
		}
	}*/
}

UInt64 g_pickValue = 0; // This gets set shortly before the below hook gets called
UInt32 g_pickedHandle = 0;

void PickLinearCastHook(hkpWorld *world, const hkpCollidable* collA, const hkpLinearCastInput* input, hkpCdPointCollector* castCollector, hkpCdPointCollector* startCollector)
{
	bool displayLeft = g_leftGrabber->ShouldDisplayRollover();
	bool displayRight = g_rightGrabber->ShouldDisplayRollover();

	if (displayRight || displayLeft) {
		return;
	}

	((_hkpWorld_LinearCast)pickLinearCastHookedFuncAddr)(world, collA, input, castCollector, startCollector);

	if (g_pickValue == 2) {
		CrosshairPickData *pickData = *g_pickData;
		if (pickData) {
			bool isLeftHanded = *g_leftHandedMode;
			g_pickedHandle = isLeftHanded ? pickData->leftHandle1 : pickData->rightHandle1;
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

		_MESSAGE("ShaderReferenceEffect ctor hook complete");
	}
	/*
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
	*/
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

				// Save r14, as it has a value that determines if we should read the selected handles or not
				push(rax);
				mov(rax, (uintptr_t)&g_pickValue);
				mov(ptr[rax], r14);
				pop(rax);

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
}
