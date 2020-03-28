#pragma once

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"


namespace Config {
	struct Options {
		float castDistance = 5.0f;
		float castRadius = 0.4f;
		float handActivateDistance = 30.0f;
		float requiredCastDotProduct = cosf(50.0f * 0.0174533);
		float hoverVelocityMultiplier = 0.17f;
		float pullVelocityMultiplier = 0.9f;
		float pushVelocityMultiplier = 0.9f;
		float bodyVelocityMultiplier = 0.6f;
		float massExponent = 0.5;
		long long selectedLeewayTime = 250; // in ms, time to keep something selected after not pointing at it anymore
		long long triggerPressedLeewayTime = 300; // in ms, time after pressing the trigger after which the trigger is considered not pressed anymore
		bool equipWeapons = false;

		NiPoint3 handAdjust = { -0.018, -0.965, 0.261 };
	};
	extern Options options; // global object containing options


	const std::string & GetConfigPath();

	std::string GetConfigOption(const char * section, const char * key);

	bool GetConfigOptionFloat(const char *section, const char *key, float *out);
	bool GetConfigOptionInt(const char *section, const char *key, int *out);
	bool GetConfigOptionBool(const char *section, const char *key, bool *out);
}
