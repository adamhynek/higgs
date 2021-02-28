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
		virtual unsigned int getBuildNumber();

		virtual void addPulledCallback(PulledCallback Callback);
		virtual void addGrabbedCallback(GrabbedCallback callback);
		virtual void addDroppedCallback(DroppedCallback callback);
		virtual void addStashedCallback(StashedCallback callback);
		virtual void addConsumedCallback(ConsumedCallback callback);
		virtual void addCollisionCallback(CollisionCallback callback);

		virtual void GrabObject(TESObjectREFR *object, bool isLeft);
		virtual TESObjectREFR * GetGrabbedObject(bool isLeft);
		virtual bool CanGrabObject(bool isLeft);
		virtual void DisableHand(bool isLeft);
		virtual void EnableHand(bool isLeft);
		virtual bool IsDisabled(bool isLeft);


		std::mutex addCallbackLock;
		std::vector<PulledCallback> pulledCallbacks;
		std::vector<GrabbedCallback> grabbedCallbacks;
		std::vector<DroppedCallback> droppedCallbacks;
		std::vector<StashedCallback> stashedCallbacks;
		std::vector<ConsumedCallback> consumedCallbacks;
		std::vector<CollisionCallback> collisionCallbacks;

		std::atomic<int> rightDisableCount = 0;
		std::atomic<int> leftDisableCount = 0;
	};

	void TriggerPulledCallbacks(bool isLeft, TESObjectREFR *pulledRefr);
	void TriggerGrabbedCallbacks(bool isLeft, TESObjectREFR *grabbedRefr);
	void TriggerDroppedCallbacks(bool isLeft, TESObjectREFR *droppedRefr);
	void TriggerStashedCallbacks(bool isLeft, TESForm *stashedForm);
	void TriggerConsumedCallbacks(bool isLeft, TESForm *consumedForm);
	void TriggerCollisionCallbacks(bool isLeft, float mass, float separatingVelocity);
}

extern HiggsPluginAPI::HiggsInterface001 g_interface001;
