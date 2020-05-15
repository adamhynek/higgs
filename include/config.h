#pragma once

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"


namespace Config {
	struct Options {
		float castDistance = 5.0f;
		float castRadius = 0.4f;
		float handActivateDistance = 30.0f;
		float requiredCastDotProduct = cosf(50.0f * 0.0174533);
		float grabbedDotProductThreshold = cosf(15.0f * 0.0174533);
		float hoverVelocityMultiplier = 0.17f;
		float pullVelocityMultiplier = 0.9f;
		float pushVelocityMultiplier = 0.01f;
		float massExponent = 0.55f;
		float inverseMassLimit = 0.1f;
		float rolloverScale = 10.0f;
		float maxItemHeight = 4.0f;
		float maxBodyHeight = 1.5f;
		float pushPullSpeedThreshold = 90.0f;
		float pullAngularSpeedThreshold = 9.0f;
		double grabbedRampUpTime = 1.0f; // in s, time over which to ramp up speed after grabbing an object
		double selectedLeewayTime = 0.25; // in s, time to keep something selected after not pointing at it anymore
		double triggerPressedLeewayTime = 0.3; // in s, time after pressing the trigger after which the trigger is considered not pressed anymore
		bool ignoreWeaponChecks = false;

		NiPoint3 handAdjust = { -0.018, -0.965, 0.261 };
	};
	extern Options options; // global object containing options


	// Fills Options struct from INI file
	bool ReadConfigOptions();

	const std::string & GetConfigPath();

	std::string GetConfigOption(const char * section, const char * key);

	bool GetConfigOptionDouble(const char *section, const char *key, double *out);
	bool GetConfigOptionFloat(const char *section, const char *key, float *out);
	bool GetConfigOptionInt(const char *section, const char *key, int *out);
	bool GetConfigOptionBool(const char *section, const char *key, bool *out);
}
