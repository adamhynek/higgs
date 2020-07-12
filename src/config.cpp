#include <chrono>
#include "utils.h"
#include "config.h"


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
		float handAdjustX, handAdjustY, handAdjustZ;
		if (!GetConfigOptionFloat("Settings", "HandAdjustX", &handAdjustX)) return false;
		if (!GetConfigOptionFloat("Settings", "HandAdjustY", &handAdjustY)) return false;
		if (!GetConfigOptionFloat("Settings", "HandAdjustZ", &handAdjustZ)) return false;

		NiPoint3 handAdjust = { handAdjustX, handAdjustY, handAdjustZ };
		if (VectorLength(handAdjust) > 0.0001f) {
			options.handAdjust = VectorNormalized(handAdjust);
		}
		else {
			_WARNING("Supplied hand adjust vector is too small");
			return false;
		}

		if (!GetConfigOptionFloat("Settings", "CastRadius", &options.castRadius)) return false;
		if (!GetConfigOptionFloat("Settings", "CastDistance", &options.castDistance)) return false;

		if (!GetConfigOptionFloat("Settings", "GrabRadius", &options.wideGrabRadius)) return false;
		if (!GetConfigOptionFloat("Settings", "WideGrabRadius", &options.wideGrabRadius)) return false;

		float castDirectionRequiredHalfAngle;
		if (!GetConfigOptionFloat("Settings", "CastDirectionRequiredHalfAngle", &castDirectionRequiredHalfAngle)) return false;
		options.requiredCastDotProduct = cosf(castDirectionRequiredHalfAngle * 0.0174533); // degrees to radians

		float grabbedAngleThreshold;
		if (!GetConfigOptionFloat("Settings", "GrabAngleThreshold", &grabbedAngleThreshold)) return false;
		options.grabbedDotProductThreshold = cosf(grabbedAngleThreshold * 0.0174533); // degrees to radians

		if (!GetConfigOptionDouble("Settings", "SelectedFadeTime", &options.selectedLeewayTime)) return false;
		if (!GetConfigOptionDouble("Settings", "TriggerPreemptTime", &options.triggerPressedLeewayTime)) return false;

		if (!GetConfigOptionFloat("Settings", "GrabStartSpeed", &options.grabStartSpeed)) return false;

		if (!GetConfigOptionBool("Settings", "IgnoreWeaponChecks", &options.ignoreWeaponChecks)) return false;

		if (!GetConfigOptionFloat("Settings", "HoverVelocityMultiplier", &options.hoverVelocityMultiplier)) return false;
		if (!GetConfigOptionFloat("Settings", "PullVelocityMultiplier", &options.pullVelocityMultiplier)) return false;
		if (!Config::GetConfigOptionFloat("Settings", "PushVelocityMultiplier", &options.pushVelocityMultiplier)) return false;
		if (!GetConfigOptionFloat("Settings", "MassExponent", &options.massExponent)) return false;
		if (!GetConfigOptionFloat("Settings", "PushPullSpeedThreshold", &options.pushPullSpeedThreshold)) return false;
		if (!GetConfigOptionFloat("Settings", "PullAngularSpeedThreshold", &options.pullAngularSpeedThreshold)) return false;

		if (!GetConfigOptionFloat("Settings", "RolloverScale", &options.rolloverScale)) return false;

		if (!GetConfigOptionFloat("Settings", "MaxItemHeight", &options.maxItemHeight)) return false;
		if (!GetConfigOptionFloat("Settings", "MaxBodyHeight", &options.maxBodyHeight)) return false;

		if (!GetConfigOptionFloat("Settings", "InverseMassLimit", &options.inverseMassLimit)) return false;

		if (!GetConfigOptionBool("Settings", "EnableTrigger", &options.enableTrigger)) return false;
		if (!GetConfigOptionBool("Settings", "EnableGrip", &options.enableGrip)) return false;

		return true;
	}

	const std::string & GetConfigPath()
	{
		static std::string s_configPath;

		if (s_configPath.empty()) {
			std::string	runtimePath = GetRuntimeDirectory();
			if (!runtimePath.empty()) {
				s_configPath = runtimePath + "Data\\SKSE\\Plugins\\forcepull_vr.ini";

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
