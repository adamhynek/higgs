#include <mutex>
#include "xbyak/xbyak.h"
#include "skse64_common/Relocation.h"
#include "skse64_common/BranchTrampoline.h"

#include "hooks.h"
#include "shaders.h"
#include "RE/havok.h"
#include "grabber.h"


auto shaderHookLoc = RelocAddr<uintptr_t>(0x2AE3E8);
auto shaderHookedFunc = RelocAddr<uintptr_t>(0x564DD0);

uintptr_t shaderHookedFuncAddr = 0;

auto worldHookLoc = RelocAddr<uintptr_t>(0x271EF9);

auto pickHookLoc = RelocAddr<uintptr_t>(0x6D2E7F);
auto pickHookedFunc = RelocAddr<uintptr_t>(0x3BA0C0); // CrosshairPickData::Pick

uintptr_t pickHookedFuncAddr = 0;


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

		if (g_rightGrabber->state == Grabber::State::Held) {
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

		if (g_leftGrabber->state == Grabber::State::Held) {
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

void PickHook()
{
	// Set handles to display in rollover text, after the game sets them
	bool isLeftHanded = *g_leftHandedMode;

	std::lock_guard<std::mutex> leftLock(g_leftGrabber->deselectLock);
	std::lock_guard<std::mutex> rightLock(g_rightGrabber->deselectLock);

	bool displayLeft = g_leftGrabber->ShouldDisplayRollover();
	bool displayRight = g_rightGrabber->ShouldDisplayRollover();

	if (displayRight || displayLeft) {
		// Something is grabbed
		if (displayRight && displayLeft) {
			// Pick whichever hand grabbed last
			if (g_leftGrabber->selectionLockedTime > g_rightGrabber->selectionLockedTime) {
				g_leftGrabber->SetSelectedHandles(isLeftHanded);
			}
			else {
				g_rightGrabber->SetSelectedHandles(isLeftHanded);
			}
		}
		else if (displayRight) {
			g_rightGrabber->SetSelectedHandles(isLeftHanded);
		}
		else if (displayLeft) {
			g_leftGrabber->SetSelectedHandles(isLeftHanded);
		}
	}
}


void PerformHooks(void)
{
	// First, set our addresses
	shaderHookedFuncAddr = shaderHookedFunc.GetUIntPtr();
	pickHookedFuncAddr = pickHookedFunc.GetUIntPtr();

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
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(worldHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("World update hook complete");
	}

	{
		struct Code : Xbyak::CodeGenerator {
			Code(void * buf) : Xbyak::CodeGenerator(256, buf)
			{
				Xbyak::Label jumpBack;

				// Original code
				mov(rax, pickHookedFuncAddr);
				call(rax);

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
				mov(rax, (uintptr_t)PickHook);
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
				dq(pickHookLoc.GetUIntPtr() + 5);
			}
		};

		void * codeBuf = g_localTrampoline.StartAlloc();
		Code code(codeBuf);
		g_localTrampoline.EndAlloc(code.getCurr());

		g_branchTrampoline.Write5Branch(pickHookLoc.GetUIntPtr(), uintptr_t(code.getCode()));

		_MESSAGE("Pick hook complete");
	}
}