#pragma once

#include <set>

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"


namespace Config {
	struct Options {
		bool debugDrawControllers = false;

		bool enableHandDelay = false;
		float roomspaceHandLinearSpeed = 10.f;
		float roomspaceHandAngularSpeed = 500.f;

		bool treatProjectileWeaponHitsAsSelfHits = true;

		float farCastDistance = 5.0f;
		float farCastRadius = 0.3f;
		float nearCastRadius = 0.05f;
		float nearCastDistance = 0.1f;
		float widePullGrabRadius = 0.9f;
		float nearbyGrabBodyRadius = 0.1f;
		float requiredCastDotProduct = cosf(50.0f * 0.0174533);
		float rolloverScale = 10.0f;
		float pullSpeedThreshold = 1.2f; // m/s
		float grabStartSpeed = 200.0f; // skyrim units/s
		float grabStartAngularSpeed = 360.0f; // deg/s
		float grabLateralWeight = 0.6f;
		float grabDirectionalWeight = 0.4f;
		float throwVelocityThreshold = 1.0f; // m/s
		float throwVelocityBoostFactor = 1.0f;
		float shoulderVelocityThreshold = 2.0f; // m/s
		float mouthVelocityThreshold = 2.0f; // m/s
		float pullDestinationZOffset = 0.01f; // in meters, z offset above the palm at which to target the pulled object
		float pulledAngularDamping = 8.0f; // angular damping to overwrite for pulled objects. This is pretty high, in order to prevent the object from spinning out of control.
		float pulledGrabHandAdjustDistance = 0.15f; // in meters, amount to move the hand back when grabbing a pulled object
		float angularVelocityMultiplier = 0.6f;
		float tangentialVelocityLimit = 5.0f;
		float twoHandedRotationSnapSpeed = 1200.0f;

		float selectedCloseFingerAnimMaxHandSpeed = 0.9f;
		float selectedCloseFingerAnimValue = 0.9f;
		float fingerAnimateGrabLinearSpeed = 4.0f;
		float fingerAnimateGrabAngularSpeed = 630.0f;
		float fingerAnimateStartLinearSpeed = 2.0f;
		float fingerAnimateStartAngularSpeed = 135.0f;
		float fingerAnimateEndLinearSpeed = 2.0f;
		float fingerAnimateEndAngularSpeed = 45.0f;

		float nearbyGrabMaxLinearVelocity = 0.2f;
		float nearbyGrabMaxAngularVelocity = 1.0f;
		float nearbyGrabLinearDamping = 500.0f;
		float nearbyGrabAngularDamping = 50.0f;

		float pullDurationA = 0.715619f;
		float pullDurationB = -0.415619f;
		float pullDurationC = 0.656256f;

		float selectionLockedStartHapticStrength = 0.15f;
		float selectionLockedStartHapticDuration = 0.05f;
		float selectionLockedEndHapticStrength = 0.1f;
		float selectionLockedEndHapticDuration = 0.02f;
		float selectionLockedBaseHapticStrength = 0.05f;
		float selectionLockedProportionalHapticStrength = 0.3f;

		float grabBaseHapticStrength = 0.25f;
		float grabProportionalHapticStrength = 0.06f;
		float grabHapticMassExponent = 0.6f;

		int collisionMaxInactiveFramesToConsiderActive = 15;
		int collisionMaxInactiveFramesBeforeCleanup = 100;
		float collisionMaxInitialContactPointDistance = 0.01f;
		float collisionMinHapticSpeed = 0.2f;
		float collisionBaseHapticStrength = 0.1f;
		float collisionSpeedProportionalHapticStrength = 0.05f;
		float collisionMassProportionalHapticStrength = 0.03f;
		float collisionHapticMassExponent = 0.6f;
		float collisionHapticDuration = 0.01f;

		float shoulderConstantHapticStrength = 0.2f;
		float shoulderDropHapticStrength = 0.5f;

		float mouthConstantHapticStrength = 0.3f;
		float mouthDropHapticStrength = 0.5f;

		float maxHandDistance = 0.7f;
		float minDampedRequiredVelocityProportion = 0.5f;
		float minVelocityToPotentiallyDamp = 1.0f;
		float dampedLinearVelocityMultiplier = 0.2f;
		float dampedLinearVelocityExponent = 0.8f;
		float dampedAngularVelocityMultiplier = 0.2f;
		float dampedCollisionHapticStrengthMultiplier = 0.4f;

		float rolloverMinAlphaToShow = 0.4f;
		float rolloverAlphaLogisticK = 30.0f;
		float rolloverAlphaLogisticMidpoint = 0.8f;
		float rolloverAlphaFadeInLogisticK = 12.0f;
		float rolloverAlphaFadeInLogisticMidpoint = 0.6f;

		float geometryVertexAlphaThreshold = 0.45f;

		double selectedLeewayTime = 0.25; // in s, time to keep something selected after not pointing at it anymore
		double triggerPressedLeewayTime = 0.3; // in s, time after pressing the trigger after which the trigger is considered not pressed anymore
		double inputLeewayTime = 0.3; // in s, time after pressing the trigger on a selected object, within which if you let go, input is retriggered
		double forceInputTime = 0.03; // in s, amount of time to force input
		double pullApplyVelocityTime = 0.2; // in s, time within which to constantly apply velocity to a pulled object when it's initially pulled
		double pullTrackHandTime = 0.1; // in s, time within which to constantly adjust the target of the pull when it's initially pulled
		double lootSpawnInTime = 0.5; // in s, amount of time to wait for a pulled looted item to spawn in before giving up
		double grabFreezeNearbyVelocityTime = 0.1; // in s, amount of time during which to zero-out velocity of objects near the grabbed object when grabbing
		double pullHapticFadeTime = 0.15; // in s, amount of time over which to fade down the haptic strength after a pull
		double grabHapticFadeTime = 0.1;
		double grabStartMaxTime = 0.5;
		double shoulderDropHapticFadeTime = 0.2;
		double mouthDropHapticFadeTime = 0.2;
		double rolloverHideTime = 0.25;
		double preDampVelocityTime = 0.1;
		double tryLeaveDampedTime = 0.1;
		double physicsGrabInitTime = 0.1;
		double fingerAnimateEndTime = 0.8;
		double fingerAnimateEndDoubleSpeedTime = 0.1;
		double afterDropFingerAnimateTime = 0.3;
		double fingerAnimateStartDoubleSpeedTime = 0.1;
		double fingerAnimateGrabDoubleSpeedTime = 0.25;
		double weaponCollisionDisableOnHitTime = 0.5;
		double weaponCollisionDisableOnHitDelay = 0.017;
		double triggerGripIconSwitchTime = 0.8;
		double rolloverAfterGrabAlphaFadeInTime = 1.0;
		double rolloverAfterDropAlphaFadeInTime = 1.0;

		int logLevel = IDebugLog::kLevel_Message;

		bool enableWeaponCollision = true;
		bool forcePhysicsGrab = false;
		bool disableDampedGrab = false;
		bool disableDampedGrabForBodies = true;
		bool disableGrabHair = true;
		bool disableGrabGeometryWithVertexAlpha = true;
		bool inheritTangentialVelocity = true;
		bool offhandAffectsTwoHandedRotation = true;

		bool updateHandsToMatchWeaponCollisionTransform = false;
		bool useVrikWeaponTransform = true;
		float weaponCollisionScale = 1.f;

		bool slowMovementWhenObjectIsHeld = true;
		float slowMovementMassProportion = 0.625f;
		float slowMovementMassExponent = 1.f;
		float slowMovementMaxReduction = 70.f;

		bool enableHavokFix = true;
		float havokMaxTimeComplexMultiplier = 1.0f;
		float havokMaxMaxTime = 0.02778f; // 36 fps

		bool enableShadowUpdateFix = true;
		int maxNumEntitiesPerSimulationIslandToCheck = 50;
		float maxDistanceOfSimulationIslandToUpdate = 25.0f; // meters

		bool disableHeadBobbingWhileGrabbed = true;
		bool disableShaders = false;
		bool disableSelectionBeam = false;
		bool disableLooting = false;
		bool disableGravityGlovesLooting = false;
		bool disableGravityGlovesLootingLiveActors = false;
		bool allowLootingNonRagdolledActors = false;
		bool allowLootingLiveActors = false;
		bool skipActivateBooks = true;
		bool disableRolloverRumble = true;
		bool alwaysShowHands = true;
		bool disableVanillaGrab = true;

		bool allowGrabWithEmptyArrowHand = false;
		bool allowGrabWithTwoHandedOffhand = false;
		bool allowDaggerTwoHanding = true;
		bool allowTwoHandingWithSpellInOffhand = true;
		bool allowGrabCurrentHorse = false;

		bool useLoudSoundGrab = false;
		bool useLoudSoundDrop = false;
		bool useLoudSoundPull = true;

		bool enableTrigger = true;
		bool enableGrip = true;
		bool delayRightGripInput = true;
		bool delayLeftGripInput = false;

		bool enableDrinkPoison = false;
		bool overrideActivateText = true;
		bool useAttachPointForInitialGrab = true;

		NiPoint3 palmVector = { -0.018, -0.965, 0.261 };
		NiPoint3 pointingVector = { 0, 0, 1 };
		NiPoint3 palmPosition = { 0, -2.4, 6 }; // in handspace, skyrim units

		NiPoint3 handCollisionBoxHalfExtents = { 0.05, 0.015, 0.09 }; // in meters
		NiPoint3 handCollisionBoxOffset = { 0, -0.005, 0.086 }; // offset from hand node, in meters
		float handCollisionBoxRadius = 0; // in meters
		NiPoint3 handCollisionBoxHalfExtentsBeast = { 0.1, 0.015, 0.2 };
		NiPoint3 handCollisionBoxOffsetBeast = { -0.007, -0.005, 0.2 };
		float handCollisionBoxRadiusBeast = 0;

		NiPoint3 rightShoulderHmdOffset = { 17.5, -5.0, -6.85 };
		NiPoint3 leftShoulderHmdOffset = { -17.5, -5.0, -6.85 };
		NiPoint3 mouthHmdOffset = { 0.0, 10.0, -9.0 };

		float rightShoulderRadius = 11.0f;
		float leftShoulderRadius = 11.0f;
		float mouthRadius = 11.0f;

		NiPoint3 selectionBeamStretch = { 0.25f, 0.022f, 0.25f };

		NiPoint3 rolloverOffsetRight = { 7, -5, -2 };
		NiPoint3 rolloverOffsetLeft = { -7, -7, -3 };
		NiPoint3 rolloverRotation = { 2.62, 0, -1.57 };

		std::string grabString = "Grab";
		std::string pullString = "Pull";
		std::string lootString = "Loot";

		std::set<std::string, std::less<>> grabNodeNameBlacklist;
	};
	extern Options options; // global object containing options

	extern std::map<std::string, float*, std::less<>> floatMap;
	extern std::map<std::string, double*, std::less<>> doubleMap;
	extern std::map<std::string, int*, std::less<>> intMap;
	extern std::map<std::string, bool*, std::less<>> boolMap;

	// Fills Options struct from INI file
	bool ReadConfigOptions();

	bool SetSettingDouble(const std::string_view& name, double val);
	bool GetSettingDouble(const std::string_view& name, double& out);

	bool ReloadIfModified();

	const std::string & GetConfigPath();

	std::string GetConfigOption(const char * section, const char * key);

	bool GetConfigOptionDouble(const char *section, const char *key, double *out);
	bool GetConfigOptionFloat(const char *section, const char *key, float *out);
	bool GetConfigOptionInt(const char *section, const char *key, int *out);
	bool GetConfigOptionBool(const char *section, const char *key, bool *out);
}
