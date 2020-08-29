#include <mutex>
#include "xbyak/xbyak.h"
#include "skse64_common/Relocation.h"
#include "skse64_common/BranchTrampoline.h"

#include "hooks.h"
#include "effects.h"
#include "RE/havok.h"
#include "grabber.h"
#include "vrikinterface001.h"

#include <Physics/Collide/Shape/Query/hkpShapeRayCastOutput.h>


auto shaderHookLoc = RelocAddr<uintptr_t>(0x2AE3E8);
auto shaderHookedFunc = RelocAddr<uintptr_t>(0x564DD0);

uintptr_t shaderHookedFuncAddr = 0;

auto effectHookLoc = RelocAddr<uintptr_t>(0x320FEC);
auto effectHookedFunc = RelocAddr<uintptr_t>(0x55DA30);

uintptr_t effectHookedFuncAddr = 0;

auto effectRotUpdateHookLoc = RelocAddr<uintptr_t>(0x55E27E);
auto effectRotUpdateHookedFunc = RelocAddr<uintptr_t>(0x5627B0);

auto worldUpdateHookLoc = RelocAddr<uintptr_t>(0x271EF9);

auto pickHookLoc = RelocAddr<uintptr_t>(0x6D2E7F);
auto pickHookedFunc = RelocAddr<uintptr_t>(0x3BA0C0); // CrosshairPickData::Pick

uintptr_t pickHookedFuncAddr = 0;

auto pickLinearCastHookLoc = RelocAddr<uintptr_t>(0x3BA3D7);
auto pickLinearCastHookedFunc = RelocAddr<uintptr_t>(0xAB5EC0); // ahkpWorld::LinearCast

uintptr_t pickLinearCastHookedFuncAddr = 0;

auto pickRayCastHookLoc = RelocAddr<uintptr_t>(0x3BA787);


ShaderReferenceEffect ** volatile g_shaderReferenceToSet = nullptr;
void HookedShaderReferenceEffectCtor(ShaderReferenceEffect *ref) // DO NOT USE THIS MEMORY YET - IT HAS JUST BEEN ALLOCATED. LET THE GAME CONSTRUCT IT IN A BIT.
{
	if (g_shaderReferenceToSet) {
		*g_shaderReferenceToSet = ref;
	}
}

ModelReferenceEffect ** volatile g_modelReferenceToSet = nullptr;
void HookedModelReferenceEffectCtor(ModelReferenceEffect *ref) // DO NOT USE THIS MEMORY YET - IT HAS JUST BEEN ALLOCATED. LET THE GAME CONSTRUCT IT IN A BIT.
{
	if (g_modelReferenceToSet) {
		*g_modelReferenceToSet = ref;
	}
}

// outPos is the pos that is the source of the 'lookAt'. The dest is the pos of the lookAtHandle.
typedef void(*_GetREFRCenterPosition)(TESObjectREFR *refr, NiPoint3 *outPos);
void HookedModelReferenceEffectUpdateRotation(TESObjectREFR *target, NiPoint3 *outPos, ModelReferenceEffect *effect)
{
	if (effect && effect->controller && effect->controller->attachRoot) {
		NiAVObject *attachRoot = effect->controller->attachRoot;
		*outPos = attachRoot->m_worldTransform.pos;

		if (effect->artObject3D) {
			effect->artObject3D->m_localTransform.pos = { 0, 0, 30 };
		}
	}
	else {
		((_GetREFRCenterPosition)effectRotUpdateHookedFunc.GetUIntPtr())(target, outPos);
	}
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
						NiTransform newTransform = handNode->m_worldTransform * g_rightGrabber->initialObjTransformHandSpace;

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
						NiTransform newTransform = handNode->m_worldTransform * g_leftGrabber->initialObjTransformHandSpace;

						UpdateKeyframedNodeTransform(n, newTransform);
					}
				}
			}
		}
	}
}


UInt8 _cdPoint[sizeof(hkpCdPoint)]; // Allocate room for a single hkpCdPoint to return from the hooked linear cast
volatile bool g_overrideRaycast = false; // The pick linearcast happens once, before the raycasts, so we know there if we want to override the raycasts that follow
void PickLinearCastHook(hkpWorld *world, const hkpCollidable* collA, const hkpLinearCastInput* input, hkpCdPointCollector* castCollector, hkpCdPointCollector* startCollector)
{
	{
		std::lock_guard<std::mutex> leftLock(g_leftGrabber->deselectLock);
		std::lock_guard<std::mutex> rightLock(g_rightGrabber->deselectLock);

		bool displayLeft = g_leftGrabber->ShouldDisplayRollover();
		bool displayRight = g_rightGrabber->ShouldDisplayRollover();

		if (displayRight || displayLeft) {
			// Something is grabbed
			if (displayRight && displayLeft) {
				// Pick whichever hand grabbed last
				if (g_leftGrabber->selectionLockedTime > g_rightGrabber->selectionLockedTime) {
					NiPointer<TESObjectREFR> obj;
					if (LookupREFRByHandle(g_leftGrabber->selectedObject.handle, obj)) {
						hkpCollidable *collidable = g_leftGrabber->selectedObject.collidable;
						if (collidable) {
							hkpCdPoint cdPoint(*collA, *collidable);
							memmove(_cdPoint, &cdPoint, sizeof(hkpCdPoint)); // stupid
							castCollector->addCdPoint(*(hkpCdPoint*)_cdPoint);
						}
					}
				}
				else {
					NiPointer<TESObjectREFR> obj;
					if (LookupREFRByHandle(g_rightGrabber->selectedObject.handle, obj)) {
						hkpCollidable *collidable = g_rightGrabber->selectedObject.collidable;
						if (collidable) {
							hkpCdPoint cdPoint(*collA, *g_rightGrabber->selectedObject.collidable);
							memmove(_cdPoint, &cdPoint, sizeof(hkpCdPoint));
							castCollector->addCdPoint(*(hkpCdPoint*)_cdPoint);
						}
					}
				}
			}
			else if (displayRight) {
				NiPointer<TESObjectREFR> obj;
				if (LookupREFRByHandle(g_rightGrabber->selectedObject.handle, obj)) {
					hkpCollidable *collidable = g_rightGrabber->selectedObject.collidable;
					if (collidable) {
						hkpCdPoint cdPoint(*collA, *g_rightGrabber->selectedObject.collidable);
						memmove(_cdPoint, &cdPoint, sizeof(hkpCdPoint));
						castCollector->addCdPoint(*(hkpCdPoint*)_cdPoint);
					}
				}
			}
			else if (displayLeft) {
				NiPointer<TESObjectREFR> obj;
				if (LookupREFRByHandle(g_leftGrabber->selectedObject.handle, obj)) {
					hkpCollidable *collidable = g_leftGrabber->selectedObject.collidable;
					if (collidable) {
						hkpCdPoint cdPoint(*collA, *collidable);
						memmove(_cdPoint, &cdPoint, sizeof(hkpCdPoint));
						castCollector->addCdPoint(*(hkpCdPoint*)_cdPoint);
					}
				}
			}

			g_overrideRaycast = true;
			return;
		}
	}

	// do the regular pick linearcast if we don't want to override it
	g_overrideRaycast = false;
	((_hkpWorld_LinearCast)pickLinearCastHookedFuncAddr)(world, collA, input, castCollector, startCollector);
}


void PickRayCastHook(hkpShapeRayCastOutput *output)
{
	if (g_overrideRaycast) {
		output->m_hitFraction = 0;
	}
}


void PerformHooks(void)
{
	// First, set our addresses
	shaderHookedFuncAddr = shaderHookedFunc.GetUIntPtr();
	effectHookedFuncAddr = effectHookedFunc.GetUIntPtr();
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
				mov(rax, (uintptr_t)HookedModelReferenceEffectCtor);
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
				mov(rax, effectHookedFuncAddr);
				call(rax);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(effectHookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(effectHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("Effect hook complete");
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
				mov(r8, rsi); // ModelReferenceEffect is in rsi, we want to pass it to our hook
				mov(rax, (uintptr_t)HookedModelReferenceEffectUpdateRotation);
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
				dq(effectRotUpdateHookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(effectRotUpdateHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("Effect rotation update hook complete");
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
				Xbyak::Label jumpBack, saveR9;

				// Save the raycast output, which is in r9
				mov(r14, r9); // this is ok because I know r14 is not used after this, but in general it's not ok

				// Original code
				call(ptr[rax + 0x40]);

				mov(rcx, r14);

				// Call our hook
				mov(rax, (uintptr_t)PickRayCastHook);
				call(rax);

				// Jump back to whence we came (+ the size of the initial branch instruction)
				jmp(ptr[rip + jumpBack]);

				L(jumpBack);
				dq(pickRayCastHookLoc.GetUIntPtr() + 5);

				L(saveR9);
				dq(0);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(pickRayCastHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("Pick ray cast hook complete");
	}
}
