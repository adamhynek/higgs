#pragma once

#include "skse64/PapyrusNativeFunctions.h"
#include "skse64/PapyrusEvents.h"

namespace PapyrusAPI
{
	// Mostly copied from PapyrusEvents.cpp, with slight modifications

	template <typename T> void SetVMValue(VMValue * val, T arg)
	{
		VMClassRegistry * registry = (*g_skyrimVM)->GetClassRegistry();
		PackValue(val, &arg, registry);
	}

	template <typename T1, typename T2>
	class EventFunctor2 : public IFunctionArguments
	{
	public:
		EventFunctor2(BSFixedString & a_eventName, T1 a_arg1, T2 a_arg2)
			: eventName(a_eventName.data), arg1(a_arg1), arg2(a_arg2) {}

		virtual bool Copy(Output * dst)
		{
			dst->Resize(2);
			SetVMValue(dst->Get(0), arg1);
			SetVMValue(dst->Get(1), arg2);

			return true;
		}

		void operator() (const EventRegistration<TESForm*> & reg)
		{
			VMClassRegistry * registry = (*g_skyrimVM)->GetClassRegistry();
			registry->QueueEvent(reg.handle, &eventName, this);
		}

	private:
		BSFixedString	eventName;
		T1				arg1;
		T2				arg2;
	};

	bool RegisterPapyrusFuncs(VMClassRegistry* registry);

	void OnPullEvent(TESObjectREFR *refr, bool isLeft);
	void OnGrabEvent(TESObjectREFR *refr, bool isLeft);
	void OnDropEvent(TESObjectREFR *refr, bool isLeft);
	void OnStashEvent(TESForm *form, bool isLeft);
	void OnConsumeEvent(TESForm *form, bool isLeft);
}
