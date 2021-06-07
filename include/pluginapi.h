#pragma once

#include <atomic>
#include <mutex>
#include <vector>

#include "skse64/PluginAPI.h"
#include "skse64/GameReferences.h"

#include "higgsinterface001.h"


namespace HiggsPluginAPI {
	// Handles skse mod messages requesting to fetch API functions from HIGGS
	void ModMessageHandler(SKSEMessagingInterface::Message * message);

	// This object provides access to HIGGS's mod support API version 1
	struct HiggsInterface001 : IHiggsInterface001
	{
		// Gets the HIGGS build number
		virtual unsigned int GetBuildNumber();

		virtual void AddPulledCallback(PulledCallback Callback);
		virtual void AddGrabbedCallback(GrabbedCallback callback);
		virtual void AddDroppedCallback(DroppedCallback callback);
		virtual void AddStashedCallback(StashedCallback callback);
		virtual void AddConsumedCallback(ConsumedCallback callback);
		virtual void AddCollisionCallback(CollisionCallback callback);

		virtual void GrabObject(TESObjectREFR *object, bool isLeft);
		virtual TESObjectREFR * GetGrabbedObject(bool isLeft);
		virtual bool CanGrabObject(bool isLeft);

		virtual void DisableHand(bool isLeft);
		virtual void EnableHand(bool isLeft);
		virtual bool IsDisabled(bool isLeft);

		virtual void DisableWeaponCollision(bool isLeft);
		virtual void EnableWeaponCollision(bool isLeft);
		virtual bool IsWeaponCollisionDisabled(bool isLeft);


		std::mutex addCallbackLock;
		std::vector<PulledCallback> pulledCallbacks;
		std::vector<GrabbedCallback> grabbedCallbacks;
		std::vector<DroppedCallback> droppedCallbacks;
		std::vector<StashedCallback> stashedCallbacks;
		std::vector<ConsumedCallback> consumedCallbacks;
		std::vector<CollisionCallback> collisionCallbacks;

		std::atomic<int> rightDisableCount = 0;
		std::atomic<int> leftDisableCount = 0;

		std::atomic<int> rightWeaponDisableCount = 0;
		std::atomic<int> leftWeaponDisableCount = 0;
	};

	void TriggerPulledCallbacks(bool isLeft, TESObjectREFR *pulledRefr);
	void TriggerGrabbedCallbacks(bool isLeft, TESObjectREFR *grabbedRefr);
	void TriggerDroppedCallbacks(bool isLeft, TESObjectREFR *droppedRefr);
	void TriggerStashedCallbacks(bool isLeft, TESForm *stashedForm);
	void TriggerConsumedCallbacks(bool isLeft, TESForm *consumedForm);
	void TriggerCollisionCallbacks(bool isLeft, float mass, float separatingVelocity);
}

extern HiggsPluginAPI::HiggsInterface001 g_interface001;
