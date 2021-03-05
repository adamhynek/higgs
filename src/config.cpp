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

	bool ReadFloat(const std::string &name, float &val)
	{
		if (!GetConfigOptionFloat("Settings", name.c_str(), &val)) {
			_WARNING("Failed to read float config option: %s", name.c_str());
			return false;
		}

		return true;
	}

	bool ReadDouble(const std::string &name, double &val)
	{
		if (!GetConfigOptionDouble("Settings", name.c_str(), &val)) {
			_WARNING("Failed to read double config option: %s", name.c_str());
			return false;
		}

		return true;
	}

	bool ReadBool(const std::string &name, bool &val)
	{
		if (!GetConfigOptionBool("Settings", name.c_str(), &val)) {
			_WARNING("Failed to read bool config option: %s", name.c_str());
			return false;
		}

		return true;
	}

	bool ReadInt(const std::string &name, int &val)
	{
		if (!GetConfigOptionInt("Settings", name.c_str(), &val)) {
			_WARNING("Failed to read int config option: %s", name.c_str());
			return false;
		}

		return true;
	}

	bool ReadString(const std::string &name, std::string &val)
	{
		std::string	data = GetConfigOption("Settings", name.c_str());
		if (data.empty()) {
			_WARNING("Failed to read str config option: %s", name.c_str());
			return false;
		}

		val = data;
		return true;
	}

	bool ReadVector(const std::string &name, NiPoint3 &vec)
	{
		if (!ReadFloat(name + "X", vec.x)) return false;
		if (!ReadFloat(name + "Y", vec.y)) return false;
		if (!ReadFloat(name + "Z", vec.z)) return false;

		return true;
	}

	bool ReadConfigOptions()
	{
		if (!ReadVector("PalmVector", options.palmVector)) return false;
		if (!ReadVector("PointingVector", options.pointingVector)) return false;
		if (!ReadVector("PalmPosition", options.palmPosition)) return false;

		if (!ReadVector("HandCollisionBoxHalfExtents", options.handCollisionBoxHalfExtents)) return false;
		if (!ReadVector("HandCollisionBoxOffset", options.handCollisionBoxOffset)) return false;
		if (!ReadFloat("HandCollisionBoxRadius", options.handCollisionBoxRadius)) return false;

		if (!ReadVector("RightShoulderHmdOffset", options.rightShoulderHmdOffset)) return false;
		if (!ReadFloat("RightShoulderRadius", options.rightShoulderRadius)) return false;

		if (!ReadVector("LeftShoulderHmdOffset", options.leftShoulderHmdOffset)) return false;
		if (!ReadFloat("LeftShoulderRadius", options.leftShoulderRadius)) return false;

		if (!ReadVector("MouthHmdOffset", options.mouthHmdOffset)) return false;
		if (!ReadFloat("MouthRadius", options.mouthRadius)) return false;

		if (!ReadVector("RolloverOffsetRight", options.rolloverOffsetRight)) return false;
		if (!ReadVector("RolloverOffsetLeft", options.rolloverOffsetLeft)) return false;
		if (!ReadVector("RolloverRotation", options.rolloverRotation)) return false;

		if (!ReadFloat("FarCastRadius", options.farCastRadius)) return false;
		if (!ReadFloat("FarCastDistance", options.farCastDistance)) return false;

		if (!ReadFloat("NearbyGrabBodyRadius", options.nearbyGrabBodyRadius)) return false;

		float castDirectionRequiredHalfAngle;
		if (!ReadFloat("CastDirectionRequiredHalfAngle", castDirectionRequiredHalfAngle)) return false;
		options.requiredCastDotProduct = cosf(castDirectionRequiredHalfAngle * 0.0174533); // degrees to radians

		float grabbedAngleThreshold;
		if (!ReadFloat("GrabAngleThreshold", grabbedAngleThreshold)) return false;
		options.grabbedDotProductThreshold = cosf(grabbedAngleThreshold * 0.0174533); // degrees to radians

		if (!ReadDouble("SelectedFadeTime", options.selectedLeewayTime)) return false;
		if (!ReadDouble("TriggerPreemptTime", options.triggerPressedLeewayTime)) return false;
		if (!ReadDouble("InputLeewayTime", options.inputLeewayTime)) return false;
		if (!ReadDouble("ForceInputTime", options.forceInputTime)) return false;
		if (!ReadDouble("PullApplyVelocityTime", options.pullApplyVelocityTime)) return false;
		if (!ReadDouble("PullTrackHandTime", options.pullTrackHandTime)) return false;
		if (!ReadDouble("LootSpawnInTime", options.lootSpawnInTime)) return false;
		if (!ReadDouble("GrabFreezeNearbyVelocityTime", options.grabFreezeNearbyVelocityTime)) return false;
		if (!ReadDouble("PullHapticFadeTime", options.pullHapticFadeTime)) return false;
		if (!ReadDouble("GrabHapticFadeTime", options.grabHapticFadeTime)) return false;
		if (!ReadDouble("GrabStartMaxTime", options.grabStartMaxTime)) return false;
		if (!ReadDouble("ShoulderDropHapticFadeTime", options.shoulderDropHapticFadeTime)) return false;
		if (!ReadDouble("MouthDropHapticFadeTime", options.mouthDropHapticFadeTime)) return false;
		if (!ReadDouble("RolloverHideTime", options.rolloverHideTime)) return false;

		if (!ReadFloat("GrabStartSpeed", options.grabStartSpeed)) return false;
		if (!ReadFloat("GrabStartAngularSpeed", options.grabStartAngularSpeed)) return false;

		if (!ReadFloat("PullSpeedThreshold", options.pullSpeedThreshold)) return false;

		if (!ReadFloat("RolloverScale", options.rolloverScale)) return false;

		if (!ReadFloat("ThrowVelocityThreshold", options.throwVelocityThreshold)) return false;
		if (!ReadFloat("ThrowVelocityBoostFactor", options.throwVelocityBoostFactor)) return false;

		if (!ReadFloat("ShoulderVelocityThreshold", options.shoulderVelocityThreshold)) return false;
		if (!ReadFloat("MouthVelocityThreshold", options.mouthVelocityThreshold)) return false;

		if (!ReadFloat("PullDestinationZOffset", options.pullDestinationZOffset)) return false;

		if (!ReadFloat("PulledAngularDamping", options.pulledAngularDamping)) return false;

		if (!ReadFloat("PulledGrabHandAdjustDistance", options.pulledGrabHandAdjustDistance)) return false;

		if (!ReadFloat("SelectionLockedBaseHapticStrength", options.selectionLockedBaseHapticStrength)) return false;
		if (!ReadFloat("SelectionLockedProportionalHapticStrength", options.selectionLockedProportionalHapticStrength)) return false;

		if (!ReadFloat("GrabBaseHapticStrength", options.grabBaseHapticStrength)) return false;
		if (!ReadFloat("GrabProportionalHapticStrength", options.grabProportionalHapticStrength)) return false;
		if (!ReadFloat("GrabHapticMassExponent", options.grabHapticMassExponent)) return false;

		if (!ReadFloat("CollisionMinHapticSpeed", options.collisionMinHapticSpeed)) return false;
		if (!ReadFloat("CollisionBaseHapticStrength", options.collisionBaseHapticStrength)) return false;
		if (!ReadFloat("CollisionMassProportionalHapticStrength", options.collisionMassProportionalHapticStrength)) return false;
		if (!ReadFloat("CollisionSpeedProportionalHapticStrength", options.collisionSpeedProportionalHapticStrength)) return false;
		if (!ReadFloat("CollisionHapticMassExponent", options.collisionHapticMassExponent)) return false;
		if (!ReadFloat("CollisionHapticDuration", options.collisionHapticDuration)) return false;

		if (!ReadFloat("ShoulderConstantHapticStrength", options.shoulderConstantHapticStrength)) return false;
		if (!ReadFloat("ShoulderDropHapticStrength", options.shoulderDropHapticStrength)) return false;

		if (!ReadFloat("MouthConstantHapticStrength", options.mouthConstantHapticStrength)) return false;
		if (!ReadFloat("MouthDropHapticStrength", options.mouthDropHapticStrength)) return false;

		if (!ReadFloat("NearbyGrabLinearDamping", options.nearbyGrabLinearDamping)) return false;
		if (!ReadFloat("NearbyGrabAngularDamping", options.nearbyGrabAngularDamping)) return false;

		if (!ReadFloat("NearbyGrabMaxLinearVelocity", options.nearbyGrabMaxLinearVelocity)) return false;
		if (!ReadFloat("NearbyGrabMaxAngularVelocity", options.nearbyGrabMaxAngularVelocity)) return false;

		if (!ReadFloat("PullDurationA", options.pullDurationA)) return false;
		if (!ReadFloat("PullDurationB", options.pullDurationB)) return false;
		if (!ReadFloat("PullDurationC", options.pullDurationC)) return false;

		if (!ReadFloat("GrabLateralWeight", options.grabLateralWeight)) return false;
		if (!ReadFloat("GrabDirectionalWeight", options.grabDirectionalWeight)) return false;

		if (!ReadBool("UseLoudSoundGrab", options.useLoudSoundGrab)) return false;
		if (!ReadBool("UseLoudSoundDrop", options.useLoudSoundDrop)) return false;
		if (!ReadBool("UseLoudSoundPull", options.useLoudSoundPull)) return false;

		if (!ReadBool("DisableShaders", options.disableShaders)) return false;
		if (!ReadBool("DisableLooting", options.disableLooting)) return false;
		if (!ReadBool("SkipActivateBooks", options.skipActivateBooks)) return false;
		if (!ReadBool("DisableRolloverRumble", options.disableRolloverRumble)) return false;
		if (!ReadBool("DisableHeadBobbingWhileGrabbed", options.disableHeadBobbingWhileGrabbed)) return false;
		if (!ReadBool("DisableFarCastWhileAimingAtNPCRight", options.disableFarCastWhileAimingAtNPCRight)) return false;
		if (!ReadBool("DisableFarCastWhileAimingAtNPCLeft", options.disableFarCastWhileAimingAtNPCLeft)) return false;

		if (!ReadBool("EnableTrigger", options.enableTrigger)) return false;
		if (!ReadBool("EnableGrip", options.enableGrip)) return false;

		if (!ReadBool("DisableTriggerWhenWeaponsSheathed", options.disableTriggerWhenWeaponsSheathed)) return false;
		if (!ReadBool("EnableDrinkPoison", options.enableDrinkPoison)) return false;
		if (!ReadBool("OverrideActivateText", options.overrideActivateText)) return false;

		if (!ReadBool("DelayRightGripInput", options.delayRightGripInput)) return false;
		if (!ReadBool("DelayLeftGripInput", options.delayLeftGripInput)) return false;

		if (!ReadString("GrabString", Config::options.grabString)) return false;
		if (!ReadString("PullString", Config::options.pullString)) return false;
		if (!ReadString("LootString", Config::options.lootString)) return false;

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

	std::string GetConfigOption(const char *section, const char *key)
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
