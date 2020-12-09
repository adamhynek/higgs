#pragma once

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"


namespace Config {
	struct Options {
		float castDistance = 5.0f;
		float castRadius = 0.4f;
		float grabRadius = 0.1f;
		float wideGrabRadius = 0.2f;
		float requiredCastDotProduct = cosf(50.0f * 0.0174533);
		float grabbedDotProductThreshold = cosf(45.0f * 0.0174533);
		float normalSnapAngle = cosf(45.0f * 0.0174533);
		float rolloverScale = 10.0f;
		float pushPullSpeedThreshold = 90.0f;
		float pullAngularSpeedThreshold = 9.0f;
		float grabStartSpeed = 250.0f; // skyrim units/s
		float grabStartAngularSpeed = 360.0f; // deg/s
		float grabLateralWeight = 0.6f;
		float grabDirectionalWeight = 0.4f;
		float throwVelocityThreshold = 1.0f; // m/s
		double selectedLeewayTime = 0.25; // in s, time to keep something selected after not pointing at it anymore
		double triggerPressedLeewayTime = 0.3; // in s, time after pressing the trigger after which the trigger is considered not pressed anymore
		bool ignoreWeaponChecks = false;
		bool enableTrigger = true;
		bool enableGrip = true;
		bool delayRightGripInput = true;
		bool delayLeftGripInput = false;

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
