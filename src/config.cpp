#include <chrono>
#include <filesystem>

#include "config.h"
#include "math_utils.h"


static inline double vlibGetSetting(const char * name) {
	Setting * setting = GetINISetting(name);
	double value;
	if (!setting)
		return -1;
	if (setting->GetDouble(&value))
		return value;
	return -1;
}


namespace Config {
	// Define extern options
	Options options;

	bool ReadConfigOptions()
	{
		float palmVectorX, palmVectorY, palmVectorZ;
		if (!GetConfigOptionFloat("Settings", "PalmVectorX", &palmVectorX)) return false;
		if (!GetConfigOptionFloat("Settings", "PalmVectorY", &palmVectorY)) return false;
		if (!GetConfigOptionFloat("Settings", "PalmVectorZ", &palmVectorZ)) return false;

		NiPoint3 palmVector = { palmVectorX, palmVectorY, palmVectorZ };
		if (VectorLength(palmVector) > 0.0001f) {
			options.palmVector = VectorNormalized(palmVector);
		}
		else {
			_WARNING("Supplied palm vector is too small");
			return false;
		}

		float pointingVectorX, pointingVectorY, pointingVectorZ;
		if (!GetConfigOptionFloat("Settings", "PointingVectorX", &pointingVectorX)) return false;
		if (!GetConfigOptionFloat("Settings", "PointingVectorY", &pointingVectorY)) return false;
		if (!GetConfigOptionFloat("Settings", "PointingVectorZ", &pointingVectorZ)) return false;

		NiPoint3 pointingVector = { pointingVectorX, pointingVectorY, pointingVectorZ };
		if (VectorLength(pointingVector) > 0.0001f) {
			options.pointingVector = VectorNormalized(pointingVector);
		}
		else {
			_WARNING("Supplied pointing vector is too small");
			return false;
		}

		float palmPositionX, palmPositionY, palmPositionZ;
		if (!GetConfigOptionFloat("Settings", "PalmPositionX", &palmPositionX)) return false;
		if (!GetConfigOptionFloat("Settings", "PalmPositionY", &palmPositionY)) return false;
		if (!GetConfigOptionFloat("Settings", "PalmPositionZ", &palmPositionZ)) return false;
		options.palmPosition = { palmPositionX, palmPositionY, palmPositionZ };

		float handCollisionBoxHalfExtentsX, handCollisionBoxHalfExtentsY, handCollisionBoxHalfExtentsZ;
		if (!GetConfigOptionFloat("Settings", "HandCollisionBoxHalfExtentsX", &handCollisionBoxHalfExtentsX)) return false;
		if (!GetConfigOptionFloat("Settings", "HandCollisionBoxHalfExtentsY", &handCollisionBoxHalfExtentsY)) return false;
		if (!GetConfigOptionFloat("Settings", "HandCollisionBoxHalfExtentsZ", &handCollisionBoxHalfExtentsZ)) return false;
		options.handCollisionBoxHalfExtents = { handCollisionBoxHalfExtentsX, handCollisionBoxHalfExtentsY, handCollisionBoxHalfExtentsZ };

		float handCollisionBoxOffsetX, handCollisionBoxOffsetY, handCollisionBoxOffsetZ;
		if (!GetConfigOptionFloat("Settings", "HandCollisionBoxOffsetX", &handCollisionBoxOffsetX)) return false;
		if (!GetConfigOptionFloat("Settings", "HandCollisionBoxOffsetY", &handCollisionBoxOffsetY)) return false;
		if (!GetConfigOptionFloat("Settings", "HandCollisionBoxOffsetZ", &handCollisionBoxOffsetZ)) return false;
		options.handCollisionBoxOffset = { handCollisionBoxOffsetX, handCollisionBoxOffsetY, handCollisionBoxOffsetZ };

		if (!GetConfigOptionFloat("Settings", "HandCollisionBoxRadius", &options.handCollisionBoxRadius)) return false;

		float rightShoulderHmdOffsetX, rightShoulderHmdOffsetY, rightShoulderHmdOffsetZ;
		if (!GetConfigOptionFloat("Settings", "RightShoulderHmdOffsetX", &rightShoulderHmdOffsetX)) return false;
		if (!GetConfigOptionFloat("Settings", "RightShoulderHmdOffsetY", &rightShoulderHmdOffsetY)) return false;
		if (!GetConfigOptionFloat("Settings", "RightShoulderHmdOffsetZ", &rightShoulderHmdOffsetZ)) return false;
		options.rightShoulderHmdOffset = { rightShoulderHmdOffsetX, rightShoulderHmdOffsetY, rightShoulderHmdOffsetZ };

		if (!GetConfigOptionFloat("Settings", "RightShoulderRadius", &options.rightShoulderRadius)) return false;

		float leftShoulderHmdOffsetX, leftShoulderHmdOffsetY, leftShoulderHmdOffsetZ;
		if (!GetConfigOptionFloat("Settings", "LeftShoulderHmdOffsetX", &leftShoulderHmdOffsetX)) return false;
		if (!GetConfigOptionFloat("Settings", "LeftShoulderHmdOffsetY", &leftShoulderHmdOffsetY)) return false;
		if (!GetConfigOptionFloat("Settings", "LeftShoulderHmdOffsetZ", &leftShoulderHmdOffsetZ)) return false;
		options.leftShoulderHmdOffset = { leftShoulderHmdOffsetX, leftShoulderHmdOffsetY, leftShoulderHmdOffsetZ };

		if (!GetConfigOptionFloat("Settings", "LeftShoulderRadius", &options.leftShoulderRadius)) return false;

		float mouthHmdOffsetX, mouthHmdOffsetY, mouthHmdOffsetZ;
		if (!GetConfigOptionFloat("Settings", "MouthHmdOffsetX", &mouthHmdOffsetX)) return false;
		if (!GetConfigOptionFloat("Settings", "MouthHmdOffsetY", &mouthHmdOffsetY)) return false;
		if (!GetConfigOptionFloat("Settings", "MouthHmdOffsetZ", &mouthHmdOffsetZ)) return false;
		options.mouthHmdOffset = { mouthHmdOffsetX, mouthHmdOffsetY, mouthHmdOffsetZ };

		if (!GetConfigOptionFloat("Settings", "MouthRadius", &options.mouthRadius)) return false;

		float rolloverOffsetRightX, rolloverOffsetRightY, rolloverOffsetRightZ;
		if (!GetConfigOptionFloat("Settings", "RolloverOffsetRightX", &rolloverOffsetRightX)) return false;
		if (!GetConfigOptionFloat("Settings", "RolloverOffsetRightY", &rolloverOffsetRightY)) return false;
		if (!GetConfigOptionFloat("Settings", "RolloverOffsetRightZ", &rolloverOffsetRightZ)) return false;
		options.rolloverOffsetRight = { rolloverOffsetRightX, rolloverOffsetRightY, rolloverOffsetRightZ };

		float rolloverOffsetLeftX, rolloverOffsetLeftY, rolloverOffsetLeftZ;
		if (!GetConfigOptionFloat("Settings", "RolloverOffsetLeftX", &rolloverOffsetLeftX)) return false;
		if (!GetConfigOptionFloat("Settings", "RolloverOffsetLeftY", &rolloverOffsetLeftY)) return false;
		if (!GetConfigOptionFloat("Settings", "RolloverOffsetLeftZ", &rolloverOffsetLeftZ)) return false;
		options.rolloverOffsetLeft = { rolloverOffsetLeftX, rolloverOffsetLeftY, rolloverOffsetLeftZ };

		float rolloverRotationX, rolloverRotationY, rolloverRotationZ;
		if (!GetConfigOptionFloat("Settings", "RolloverRotationX", &rolloverRotationX)) return false;
		if (!GetConfigOptionFloat("Settings", "RolloverRotationY", &rolloverRotationY)) return false;
		if (!GetConfigOptionFloat("Settings", "RolloverRotationZ", &rolloverRotationZ)) return false;
		options.rolloverRotation = { rolloverRotationX, rolloverRotationY, rolloverRotationZ };

		if (!GetConfigOptionFloat("Settings", "FarCastRadius", &options.farCastRadius)) return false;
		if (!GetConfigOptionFloat("Settings", "FarCastDistance", &options.farCastDistance)) return false;

		if (!GetConfigOptionFloat("Settings", "NearbyGrabBodyRadius", &options.nearbyGrabBodyRadius)) return false;

		float castDirectionRequiredHalfAngle;
		if (!GetConfigOptionFloat("Settings", "CastDirectionRequiredHalfAngle", &castDirectionRequiredHalfAngle)) return false;
		options.requiredCastDotProduct = cosf(castDirectionRequiredHalfAngle * 0.0174533); // degrees to radians

		float grabbedAngleThreshold;
		if (!GetConfigOptionFloat("Settings", "GrabAngleThreshold", &grabbedAngleThreshold)) return false;
		options.grabbedDotProductThreshold = cosf(grabbedAngleThreshold * 0.0174533); // degrees to radians

		if (!GetConfigOptionDouble("Settings", "SelectedFadeTime", &options.selectedLeewayTime)) return false;
		if (!GetConfigOptionDouble("Settings", "TriggerPreemptTime", &options.triggerPressedLeewayTime)) return false;
		if (!GetConfigOptionDouble("Settings", "InputLeewayTime", &options.inputLeewayTime)) return false;
		if (!GetConfigOptionDouble("Settings", "ForceInputTime", &options.forceInputTime)) return false;
		if (!GetConfigOptionDouble("Settings", "PullApplyVelocityTime", &options.pullApplyVelocityTime)) return false;
		if (!GetConfigOptionDouble("Settings", "PullTrackHandTime", &options.pullTrackHandTime)) return false;
		if (!GetConfigOptionDouble("Settings", "LootSpawnInTime", &options.lootSpawnInTime)) return false;
		if (!GetConfigOptionDouble("Settings", "GrabFreezeNearbyVelocityTime", &options.grabFreezeNearbyVelocityTime)) return false;
		if (!GetConfigOptionDouble("Settings", "PullHapticFadeTime", &options.pullHapticFadeTime)) return false;
		if (!GetConfigOptionDouble("Settings", "GrabHapticFadeTime", &options.grabHapticFadeTime)) return false;
		if (!GetConfigOptionDouble("Settings", "GrabStartMaxTime", &options.grabStartMaxTime)) return false;
		if (!GetConfigOptionDouble("Settings", "ShoulderDropHapticFadeTime", &options.shoulderDropHapticFadeTime)) return false;
		if (!GetConfigOptionDouble("Settings", "MouthDropHapticFadeTime", &options.mouthDropHapticFadeTime)) return false;
		if (!GetConfigOptionDouble("Settings", "RolloverHideTime", &options.rolloverHideTime)) return false;

		if (!GetConfigOptionFloat("Settings", "GrabStartSpeed", &options.grabStartSpeed)) return false;
		if (!GetConfigOptionFloat("Settings", "GrabStartAngularSpeed", &options.grabStartAngularSpeed)) return false;

		if (!GetConfigOptionFloat("Settings", "PullSpeedThreshold", &options.pullSpeedThreshold)) return false;

		if (!GetConfigOptionFloat("Settings", "RolloverScale", &options.rolloverScale)) return false;

		if (!GetConfigOptionFloat("Settings", "ThrowVelocityThreshold", &options.throwVelocityThreshold)) return false;
		if (!GetConfigOptionFloat("Settings", "ThrowVelocityBoostFactor", &options.throwVelocityBoostFactor)) return false;

		if (!GetConfigOptionFloat("Settings", "ShoulderVelocityThreshold", &options.shoulderVelocityThreshold)) return false;
		if (!GetConfigOptionFloat("Settings", "MouthVelocityThreshold", &options.mouthVelocityThreshold)) return false;

		if (!GetConfigOptionFloat("Settings", "PullDestinationZOffset", &options.pullDestinationZOffset)) return false;

		if (!GetConfigOptionFloat("Settings", "PulledAngularDamping", &options.pulledAngularDamping)) return false;

		if (!GetConfigOptionFloat("Settings", "PulledGrabHandAdjustDistance", &options.pulledGrabHandAdjustDistance)) return false;

		if (!GetConfigOptionFloat("Settings", "SelectionLockedBaseHapticStrength", &options.selectionLockedBaseHapticStrength)) return false;
		if (!GetConfigOptionFloat("Settings", "SelectionLockedProportionalHapticStrength", &options.selectionLockedProportionalHapticStrength)) return false;
		if (!GetConfigOptionFloat("Settings", "GrabBaseHapticStrength", &options.grabBaseHapticStrength)) return false;
		if (!GetConfigOptionFloat("Settings", "GrabProportionalHapticStrength", &options.grabProportionalHapticStrength)) return false;
		if (!GetConfigOptionFloat("Settings", "GrabHapticMassExponent", &options.grabHapticMassExponent)) return false;

		if (!GetConfigOptionFloat("Settings", "ShoulderConstantHapticStrength", &options.shoulderConstantHapticStrength)) return false;
		if (!GetConfigOptionFloat("Settings", "ShoulderDropHapticStrength", &options.shoulderDropHapticStrength)) return false;

		if (!GetConfigOptionFloat("Settings", "MouthConstantHapticStrength", &options.mouthConstantHapticStrength)) return false;
		if (!GetConfigOptionFloat("Settings", "MouthDropHapticStrength", &options.mouthDropHapticStrength)) return false;

		if (!GetConfigOptionFloat("Settings", "NearbyGrabLinearDamping", &options.nearbyGrabLinearDamping)) return false;
		if (!GetConfigOptionFloat("Settings", "NearbyGrabAngularDamping", &options.nearbyGrabAngularDamping)) return false;

		if (!GetConfigOptionFloat("Settings", "NearbyGrabMaxLinearVelocity", &options.nearbyGrabMaxLinearVelocity)) return false;
		if (!GetConfigOptionFloat("Settings", "NearbyGrabMaxAngularVelocity", &options.nearbyGrabMaxAngularVelocity)) return false;

		if (!GetConfigOptionFloat("Settings", "PullDurationA", &options.pullDurationA)) return false;
		if (!GetConfigOptionFloat("Settings", "PullDurationB", &options.pullDurationB)) return false;
		if (!GetConfigOptionFloat("Settings", "PullDurationC", &options.pullDurationC)) return false;

		if (!GetConfigOptionFloat("Settings", "GrabLateralWeight", &options.grabLateralWeight)) return false;
		if (!GetConfigOptionFloat("Settings", "GrabDirectionalWeight", &options.grabDirectionalWeight)) return false;

		if (!GetConfigOptionBool("Settings", "UseLoudSoundGrab", &options.useLoudSoundGrab)) return false;
		if (!GetConfigOptionBool("Settings", "UseLoudSoundDrop", &options.useLoudSoundDrop)) return false;
		if (!GetConfigOptionBool("Settings", "UseLoudSoundPull", &options.useLoudSoundPull)) return false;

		if (!GetConfigOptionBool("Settings", "DisableShaders", &options.disableShaders)) return false;
		if (!GetConfigOptionBool("Settings", "DisableLooting", &options.disableLooting)) return false;
		if (!GetConfigOptionBool("Settings", "SkipActivateBooks", &options.skipActivateBooks)) return false;
		if (!GetConfigOptionBool("Settings", "DisableHeadBobbingWhileGrabbed", &options.disableHeadBobbingWhileGrabbed)) return false;
		if (!GetConfigOptionBool("Settings", "DisableFarCastWhileAimingAtNPCRight", &options.disableFarCastWhileAimingAtNPCRight)) return false;
		if (!GetConfigOptionBool("Settings", "DisableFarCastWhileAimingAtNPCLeft", &options.disableFarCastWhileAimingAtNPCLeft)) return false;

		if (!GetConfigOptionBool("Settings", "EnableTrigger", &options.enableTrigger)) return false;
		if (!GetConfigOptionBool("Settings", "EnableGrip", &options.enableGrip)) return false;

		if (!GetConfigOptionBool("Settings", "DisableTriggerWhenWeaponsSheathed", &options.disableTriggerWhenWeaponsSheathed)) return false;

		if (!GetConfigOptionBool("Settings", "DelayRightGripInput", &options.delayRightGripInput)) return false;
		if (!GetConfigOptionBool("Settings", "DelayLeftGripInput", &options.delayLeftGripInput)) return false;

		return true;
	}

	bool ReloadIfModified()
	{
		namespace fs = std::filesystem;

		static long long lastModifiedConfigTime = 0;

		const std::string &path = GetConfigPath();
		auto ftime = fs::last_write_time(path);
		auto time = ftime.time_since_epoch().count();
		if (time > lastModifiedConfigTime) {
			lastModifiedConfigTime = time;

			// Reload config if file has been modified since we last read it
			if (Config::ReadConfigOptions()) {
				_MESSAGE("Successfully reloaded config parameters");
			}
			else {
				_WARNING("[WARNING] Failed to reload config options");
			}

			return true;
		}

		return false;
	}

	const std::string & GetConfigPath()
	{
		static std::string s_configPath;

		if (s_configPath.empty()) {
			std::string	runtimePath = GetRuntimeDirectory();
			if (!runtimePath.empty()) {
				s_configPath = runtimePath + "Data\\SKSE\\Plugins\\higgs_vr.ini";

				_MESSAGE("config path = %s", s_configPath.c_str());
			}
		}

		return s_configPath;
	}

	std::string GetConfigOption(const char * section, const char * key)
	{
		std::string	result;

		const std::string & configPath = GetConfigPath();
		if (!configPath.empty()) {
			char	resultBuf[256];
			resultBuf[0] = 0;

			UInt32	resultLen = GetPrivateProfileString(section, key, NULL, resultBuf, sizeof(resultBuf), configPath.c_str());

			result = resultBuf;
		}

		return result;
	}

	bool GetConfigOptionDouble(const char *section, const char *key, double *out)
	{
		std::string	data = GetConfigOption(section, key);
		if (data.empty())
			return false;

		*out = std::stod(data);
		return true;
	}

	bool GetConfigOptionFloat(const char *section, const char *key, float *out)
	{
		std::string	data = GetConfigOption(section, key);
		if (data.empty())
			return false;

		*out = std::stof(data);
		return true;
	}

	bool GetConfigOptionInt(const char *section, const char *key, int *out)
	{
		std::string	data = GetConfigOption(section, key);
		if (data.empty())
			return false;

		*out = std::stoi(data);
		return true;
	}

	bool GetConfigOptionBool(const char *section, const char *key, bool *out)
	{
		std::string	data = GetConfigOption(section, key);
		if (data.empty())
			return false;

		int val = std::stoi(data);
		if (val == 1) {
			*out = true;
			return true;
		}
		else if (val == 0) {
			*out = false;
			return true;
		}
		else {
			return false;
		}
	}
}
