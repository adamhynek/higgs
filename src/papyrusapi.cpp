#include "papyrusapi.h"
#include "grabber.h"

namespace PapyrusAPI
{
	void PapyrusGrabObject(StaticFunctionTag *base, TESObjectREFR *object, bool isLeft)
	{
		Grabber *grabber = isLeft ? g_leftGrabber : g_rightGrabber;

		grabber->externalGrabRequestedObject = object;
		grabber->externalGrabRequested = true;
	}

	TESObjectREFR *PapyrusGetGrabbedObject(StaticFunctionTag *base, bool isLeft)
	{
		Grabber *grabber = isLeft ? g_leftGrabber : g_rightGrabber;

		if (grabber->HasHeldObject()) {
			UInt32 handle = grabber->selectedObject.handle;
			// To be somewhat thread-safe, check again if we're in held after getting the handle
			if (grabber->HasHeldObject()) {
				NiPointer<TESObjectREFR> grabbedObj;
				if (LookupREFRByHandle(handle, grabbedObj)) {
					return grabbedObj;
				}
			}
		}

		return nullptr;
	}

	bool PapyrusCanGrabObject(StaticFunctionTag *base, bool isLeft)
	{
		Grabber *grabber = isLeft ? g_leftGrabber : g_rightGrabber;
		return grabber->CanGrabObject();
	}

	std::atomic<int> rightDisableCount = 0;
	std::atomic<int> leftDisableCount = 0;
	void PapyrusDisableHand(StaticFunctionTag *base, bool isLeft)
	{
		if (isLeft) {
			leftDisableCount++;
		}
		else {
			rightDisableCount++;
		}
	}

	void PapyrusEnableHand(StaticFunctionTag *base, bool isLeft)
	{
		if (isLeft) {
			leftDisableCount--;
		}
		else {
			rightDisableCount--;
		}
	}

	bool IsDisabled(bool isLeft)
	{
		if (isLeft) {
			return leftDisableCount > 0;
		}
		else {
			return rightDisableCount > 0;
		}
	}

	bool PapyrusIsDisabled(StaticFunctionTag *base, bool isLeft)
	{
		return IsDisabled(isLeft);
	}

	BSFixedString dropEventName("OnObjectDropped");
	RegistrationSetHolder<TESForm*> g_dropEventRegs;
	void RegisterForDropEvent(StaticFunctionTag *base, TESForm* object)
	{
		if (!object)
		{
			_WARNING("[WARNING] Attempt to register for drop event with null parameter");
			return;
		}
		g_dropEventRegs.Register(object->GetFormType(), object);

		if (object && object->formID)
			_MESSAGE("%d registered for drop event", object->formID);
	}

	void UnregisterForDropEvent(StaticFunctionTag *base, TESForm* object)
	{
		if (!object)
		{
			_WARNING("[WARNING] Attempt to unregister for drop event with null parameter");
			return;
		}
		g_dropEventRegs.Unregister(object->GetFormType(), object);

		if (object && object->formID)
			_MESSAGE("%d unregistered for drop event", object->formID);
	}

	void OnDropEvent(TESObjectREFR *refr, bool isLeft)
	{
		_MESSAGE("Papyrus drop event");
		// Notify Papyrus scripts
		if (g_dropEventRegs.m_data.size() > 0)
			g_dropEventRegs.ForEach(
				EventFunctor2<TESObjectREFR *, bool>(dropEventName, refr, isLeft)
			);
	}

	bool RegisterPapyrusFuncs(VMClassRegistry* registry)
	{
		registry->RegisterFunction(new NativeFunction2 <StaticFunctionTag, void, TESObjectREFR*, bool>("GrabObject", "HiggsVR", PapyrusGrabObject, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, TESObjectREFR*, bool>("GetGrabbedObject", "HiggsVR", PapyrusGetGrabbedObject, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, bool, bool>("CanGrabObject", "HiggsVR", PapyrusCanGrabObject, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, bool>("DisableHand", "HiggsVR", PapyrusDisableHand, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, bool>("EnableHand", "HiggsVR", PapyrusEnableHand, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, bool, bool>("IsDisabled", "HiggsVR", PapyrusIsDisabled, registry));

		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, TESForm*>("RegisterForDropEvent", "HiggsVR", RegisterForDropEvent, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, TESForm*>("UnregisterForDropEvent", "HiggsVR", UnregisterForDropEvent, registry));

		return true;
	}
}
