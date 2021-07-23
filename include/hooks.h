#pragma once

extern UInt32 g_pickedHandle;
extern bool g_overrideActivateText;
extern std::string g_overrideActivateTextStr;
extern void *g_wsActivateRollover;
extern bool g_overrideActivateButton;
extern std::string g_overrideActivateButtonStr;

void PerformHooks();
