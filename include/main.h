#pragma once

#include "skse64/PluginAPI.h"

void Update();

extern bool g_isVrikPresent;
extern int g_savedShadowUpdateFrameDelay;
extern int g_shadowUpdateFrame;
extern int g_numShadowUpdates;
