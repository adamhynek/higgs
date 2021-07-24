#pragma once

#include "skse64/PluginAPI.h"

void Update();

extern SKSETaskInterface *g_taskInterface;
extern bool g_isVrikPresent;
extern SInt32 g_controllerType;
extern bool g_isActivateBoundToGrip;
extern int g_savedShadowUpdateFrameDelay;
extern int g_shadowUpdateFrame;
extern int g_numShadowUpdates;
