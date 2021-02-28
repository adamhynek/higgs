#pragma once
#include "skse64/PluginAPI.h"
#include "skse64/GameReferences.h"

namespace HiggsPluginAPI {

	// Returns an IHiggsInterface001 object compatible with the API shown below
	// This should only be called after SKSE sends kMessage_PostLoad to your plugin
	struct IHiggsInterface001;
	IHiggsInterface001 * getHiggsInterface001(const PluginHandle & pluginHandle, SKSEMessagingInterface * messagingInterface);

	// This object provides access to HIGGS's mod support API
	struct IHiggsInterface001
	{
		// Gets the HIGGS build number
		virtual unsigned int getBuildNumber() = 0;

		typedef void(*PulledCallback)(bool isLeft, TESObjectREFR *pulledRefr);
		virtual void addPulledCallback(PulledCallback callback) = 0;

		typedef void(*GrabbedCallback)(bool isLeft, TESObjectREFR *grabbedRefr);
		virtual void addGrabbedCallback(GrabbedCallback callback) = 0;

		typedef void(*DroppedCallback)(bool isLeft, TESObjectREFR *droppedRefr);
		virtual void addDroppedCallback(DroppedCallback callback) = 0;

		typedef void(*StashedCallback)(bool isLeft, TESForm *stashedForm);
		virtual void addStashedCallback(StashedCallback callback) = 0;

		typedef void(*ConsumedCallback)(bool isLeft, TESForm *consumedForm);
		virtual void addConsumedCallback(ConsumedCallback callback) = 0;

		typedef void(*CollisionCallback)(bool isLeft, float mass, float separatingVelocity);
		virtual void addCollisionCallback(CollisionCallback callback) = 0;

		virtual void GrabObject(TESObjectREFR *object, bool isLeft) = 0;
		virtual TESObjectREFR * GetGrabbedObject(bool isLeft) = 0;
		virtual bool CanGrabObject(bool isLeft) = 0;
		virtual void DisableHand(bool isLeft) = 0;
		virtual void EnableHand(bool isLeft) = 0;
		virtual bool IsDisabled(bool isLeft) = 0;
	};
}

extern HiggsPluginAPI::IHiggsInterface001 * g_higgsInterface;
