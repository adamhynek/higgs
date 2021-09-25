#include "papyrusapi.h"
#include "pluginapi.h"
#include "hand.h"

namespace PapyrusAPI
{
	void PapyrusGrabObject(StaticFunctionTag *base, TESObjectREFR *object, bool isLeft) {
		g_interface001.GrabObject(object, isLeft);
	}

	TESObjectREFR *PapyrusGetGrabbedObject(StaticFunctionTag *base, bool isLeft) {
		return g_interface001.GetGrabbedObject(isLeft);
	}

	bool PapyrusCanGrabObject(StaticFunctionTag *base, bool isLeft) {
		return g_interface001.CanGrabObject(isLeft);
	}

	void PapyrusDisableHand(StaticFunctionTag *base, bool isLeft) {
		g_interface001.DisableHand(isLeft);
	}

	void PapyrusEnableHand(StaticFunctionTag *base, bool isLeft) {
		g_interface001.EnableHand(isLeft);
	}

	bool PapyrusIsDisabled(StaticFunctionTag *base, bool isLeft) {
		return g_interface001.IsDisabled(isLeft);
	}

	void PapyrusDisableWeaponCollision(StaticFunctionTag *base, bool isLeft) {
		g_interface001.DisableWeaponCollision(isLeft);
	}

	void PapyrusEnableWeaponCollision(StaticFunctionTag *base, bool isLeft) {
		g_interface001.EnableWeaponCollision(isLeft);
	}

	bool PapyrusIsWeaponCollisionDisabled(StaticFunctionTag *base, bool isLeft) {
		return g_interface001.IsWeaponCollisionDisabled(isLeft);
	}

	bool PapyrusIsTwoHanding(StaticFunctionTag *base) {
		return g_interface001.IsTwoHanding();
	}

	RegistrationSetHolder<TESForm*> g_pullEventRegs;
	void RegisterForPullEvent(StaticFunctionTag *base, TESForm* object) {
		if (!object) {
			_WARNING("[WARNING] Attempt to register for pull event with null parameter");
			return;
		}
		g_pullEventRegs.Register(object->GetFormType(), object);

		if (object && object->formID)
			_MESSAGE("%d registered for pull event", object->formID);
	}
	void UnregisterForPullEvent(StaticFunctionTag *base, TESForm* object) {
		if (!object) {
			_WARNING("[WARNING] Attempt to unregister for pull event with null parameter");
			return;
		}
		g_pullEventRegs.Unregister(object->GetFormType(), object);

		if (object && object->formID)
			_MESSAGE("%d unregistered for pull event", object->formID);
	}

	RegistrationSetHolder<TESForm*> g_grabEventRegs;
	void RegisterForGrabEvent(StaticFunctionTag *base, TESForm* object) {
		if (!object) {
			_WARNING("[WARNING] Attempt to register for grab event with null parameter");
			return;
		}
		g_grabEventRegs.Register(object->GetFormType(), object);

		if (object && object->formID)
			_MESSAGE("%d registered for grab event", object->formID);
	}
	void UnregisterForGrabEvent(StaticFunctionTag *base, TESForm* object) {
		if (!object) {
			_WARNING("[WARNING] Attempt to unregister for grab event with null parameter");
			return;
		}
		g_grabEventRegs.Unregister(object->GetFormType(), object);

		if (object && object->formID)
			_MESSAGE("%d unregistered for grab event", object->formID);
	}

	RegistrationSetHolder<TESForm*> g_dropEventRegs;
	void RegisterForDropEvent(StaticFunctionTag *base, TESForm* object) {
		if (!object) {
			_WARNING("[WARNING] Attempt to register for drop event with null parameter");
			return;
		}
		g_dropEventRegs.Register(object->GetFormType(), object);

		if (object && object->formID)
			_MESSAGE("%d registered for drop event", object->formID);
	}
	void UnregisterForDropEvent(StaticFunctionTag *base, TESForm* object) {
		if (!object) {
			_WARNING("[WARNING] Attempt to unregister for drop event with null parameter");
			return;
		}
		g_dropEventRegs.Unregister(object->GetFormType(), object);

		if (object && object->formID)
			_MESSAGE("%d unregistered for drop event", object->formID);
	}

	RegistrationSetHolder<TESForm*> g_stashEventRegs;
	void RegisterForStashEvent(StaticFunctionTag *base, TESForm* object) {
		if (!object) {
			_WARNING("[WARNING] Attempt to register for stash event with null parameter");
			return;
		}
		g_stashEventRegs.Register(object->GetFormType(), object);

		if (object && object->formID)
			_MESSAGE("%d registered for stash event", object->formID);
	}
	void UnregisterForStashEvent(StaticFunctionTag *base, TESForm* object) {
		if (!object) {
			_WARNING("[WARNING] Attempt to unregister for stash event with null parameter");
			return;
		}
		g_stashEventRegs.Unregister(object->GetFormType(), object);

		if (object && object->formID)
			_MESSAGE("%d unregistered for stash event", object->formID);
	}

	RegistrationSetHolder<TESForm*> g_consumeEventRegs;
	void RegisterForConsumeEvent(StaticFunctionTag *base, TESForm* object) {
		if (!object) {
			_WARNING("[WARNING] Attempt to register for consume event with null parameter");
			return;
		}
		g_consumeEventRegs.Register(object->GetFormType(), object);

		if (object && object->formID)
			_MESSAGE("%d registered for consume event", object->formID);
	}
	void UnregisterForConsumeEvent(StaticFunctionTag *base, TESForm* object) {
		if (!object) {
			_WARNING("[WARNING] Attempt to unregister for consume event with null parameter");
			return;
		}
		g_consumeEventRegs.Unregister(object->GetFormType(), object);

		if (object && object->formID)
			_MESSAGE("%d unregistered for consume event", object->formID);
	}

	RegistrationSetHolder<TESForm*> g_startTwoHandingEventRegs;
	void RegisterForStartTwoHandingEvent(StaticFunctionTag *base, TESForm* object) {
		if (!object) {
			_WARNING("[WARNING] Attempt to register for start two handing event with null parameter");
			return;
		}
		g_startTwoHandingEventRegs.Register(object->GetFormType(), object);

		if (object && object->formID)
			_MESSAGE("%d registered for start two handing event", object->formID);
	}
	void UnregisterForStartTwoHandingEvent(StaticFunctionTag *base, TESForm* object) {
		if (!object) {
			_WARNING("[WARNING] Attempt to unregister for start two handing event with null parameter");
			return;
		}
		g_startTwoHandingEventRegs.Unregister(object->GetFormType(), object);

		if (object && object->formID)
			_MESSAGE("%d unregistered for start two handing event", object->formID);
	}

	RegistrationSetHolder<TESForm*> g_stopTwoHandingEventRegs;
	void RegisterForStopTwoHandingEvent(StaticFunctionTag *base, TESForm* object) {
		if (!object) {
			_WARNING("[WARNING] Attempt to register for stop two handing event with null parameter");
			return;
		}
		g_stopTwoHandingEventRegs.Register(object->GetFormType(), object);

		if (object && object->formID)
			_MESSAGE("%d registered for stop two handing event", object->formID);
	}
	void UnregisterForStopTwoHandingEvent(StaticFunctionTag *base, TESForm* object) {
		if (!object) {
			_WARNING("[WARNING] Attempt to unregister for stop two handing event with null parameter");
			return;
		}
		g_stopTwoHandingEventRegs.Unregister(object->GetFormType(), object);

		if (object && object->formID)
			_MESSAGE("%d unregistered for stop two handing event", object->formID);
	}

	void OnPullEvent(TESObjectREFR *refr, bool isLeft) {
		if (g_pullEventRegs.m_data.size() > 0) {
			_MESSAGE("Papyrus pull event");
			static BSFixedString eventName("OnObjectPulled");
			g_pullEventRegs.ForEach(
				EventFunctor2<TESObjectREFR *, bool>(eventName, refr, isLeft)
			);
		}
	}

	void OnGrabEvent(TESObjectREFR *refr, bool isLeft) {
		if (g_grabEventRegs.m_data.size() > 0) {
			_MESSAGE("Papyrus grab event");
			static BSFixedString eventName("OnObjectGrabbed");
			g_grabEventRegs.ForEach(
				EventFunctor2<TESObjectREFR *, bool>(eventName, refr, isLeft)
			);
		}
	}

	void OnDropEvent(TESObjectREFR *refr, bool isLeft) {
		if (g_dropEventRegs.m_data.size() > 0) {
			_MESSAGE("Papyrus drop event");
			static BSFixedString eventName("OnObjectDropped");
			g_dropEventRegs.ForEach(
				EventFunctor2<TESObjectREFR *, bool>(eventName, refr, isLeft)
			);
		}
	}

	void OnStashEvent(TESForm *form, bool isLeft) {
		if (g_stashEventRegs.m_data.size() > 0) {
			_MESSAGE("Papyrus stash event");
			static BSFixedString eventName("OnObjectStashed");
			g_stashEventRegs.ForEach(
				EventFunctor2<TESForm *, bool>(eventName, form, isLeft)
			);
		}
	}

	void OnConsumeEvent(TESForm *form, bool isLeft) {
		if (g_consumeEventRegs.m_data.size() > 0) {
			_MESSAGE("Papyrus consume event");
			static BSFixedString eventName("OnObjectConsumed");
			g_consumeEventRegs.ForEach(
				EventFunctor2<TESForm *, bool>(eventName, form, isLeft)
			);
		}
	}

	void OnStartTwoHandingEvent() {
		if (g_startTwoHandingEventRegs.m_data.size() > 0) {
			_MESSAGE("Papyrus start two handing event");
			static BSFixedString eventName("OnStartTwoHanding");
			g_startTwoHandingEventRegs.ForEach(
				EventFunctor0(eventName)
			);
		}
	}

	void OnStopTwoHandingEvent() {
		if (g_stopTwoHandingEventRegs.m_data.size() > 0) {
			_MESSAGE("Papyrus stop two handing event");
			static BSFixedString eventName("OnStopTwoHanding");
			g_stopTwoHandingEventRegs.ForEach(
				EventFunctor0(eventName)
			);
		}
	}

	bool RegisterPapyrusFuncs(VMClassRegistry* registry) {
		registry->RegisterFunction(new NativeFunction2 <StaticFunctionTag, void, TESObjectREFR*, bool>("GrabObject", "HiggsVR", PapyrusGrabObject, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, TESObjectREFR*, bool>("GetGrabbedObject", "HiggsVR", PapyrusGetGrabbedObject, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, bool, bool>("CanGrabObject", "HiggsVR", PapyrusCanGrabObject, registry));

		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, bool>("DisableHand", "HiggsVR", PapyrusDisableHand, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, bool>("EnableHand", "HiggsVR", PapyrusEnableHand, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, bool, bool>("IsDisabled", "HiggsVR", PapyrusIsDisabled, registry));

		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, bool>("DisableWeaponCollision", "HiggsVR", PapyrusDisableWeaponCollision, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, bool>("EnableWeaponCollision", "HiggsVR", PapyrusEnableWeaponCollision, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, bool, bool>("IsWeaponCollisionDisabled", "HiggsVR", PapyrusIsWeaponCollisionDisabled, registry));

		registry->RegisterFunction(new NativeFunction0 <StaticFunctionTag, bool>("IsTwoHanding", "HiggsVR", PapyrusIsTwoHanding, registry));

		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, TESForm*>("RegisterForPullEvent", "HiggsVR", RegisterForPullEvent, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, TESForm*>("UnregisterForPullEvent", "HiggsVR", UnregisterForPullEvent, registry));

		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, TESForm*>("RegisterForGrabEvent", "HiggsVR", RegisterForGrabEvent, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, TESForm*>("UnregisterForGrabEvent", "HiggsVR", UnregisterForGrabEvent, registry));

		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, TESForm*>("RegisterForDropEvent", "HiggsVR", RegisterForDropEvent, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, TESForm*>("UnregisterForDropEvent", "HiggsVR", UnregisterForDropEvent, registry));

		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, TESForm*>("RegisterForStashEvent", "HiggsVR", RegisterForStashEvent, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, TESForm*>("UnregisterForStashEvent", "HiggsVR", UnregisterForStashEvent, registry));

		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, TESForm*>("RegisterForConsumeEvent", "HiggsVR", RegisterForConsumeEvent, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, TESForm*>("UnregisterForConsumeEvent", "HiggsVR", UnregisterForConsumeEvent, registry));

		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, TESForm*>("RegisterForStartTwoHandingEvent", "HiggsVR", RegisterForStartTwoHandingEvent, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, TESForm*>("UnregisterForStartTwoHandingEvent", "HiggsVR", UnregisterForStartTwoHandingEvent, registry));

		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, TESForm*>("RegisterForStopTwoHandingEvent", "HiggsVR", RegisterForStopTwoHandingEvent, registry));
		registry->RegisterFunction(new NativeFunction1 <StaticFunctionTag, void, TESForm*>("UnregisterForStopTwoHandingEvent", "HiggsVR", UnregisterForStopTwoHandingEvent, registry));

		return true;
	}
}
