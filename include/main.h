#pragma once

#include "skse64/PluginAPI.h"

void Update();
void LateMainThreadUpdate();
void PlayerPostApplyMovementDeltaUpdate();
void ProcessPlayerProxyCastCollector(hkpAllCdPointCollector *collector);

struct DebugTransform
{
    NiTransform transform;
    NiColorA color;
};
void RegisterDebugTransform(const std::string_view &name, const DebugTransform &transform);
void UnregisterDebugTransform(const std::string_view &name);
void DebugDraw();

extern SKSETaskInterface *g_taskInterface;
extern bool g_isVrikPresent;
extern SInt32 g_controllerType;
extern bool g_isActivateBoundToGrip;
extern int g_savedShadowUpdateFrameDelay;
extern int g_shadowUpdateFrame;
extern int g_numShadowUpdates;

extern NiPoint3 g_prevDeltaVelocity;

extern std::set<NiPointer<bhkRigidBody>> g_playerSpaceBodies;
void RegisterPlayerSpaceBody(bhkRigidBody *body, bool allowWarp = true);

extern float g_totalMassThisFrame;
void RegisterObjectMass(hkpRigidBody *body, std::optional<float> massOverride = {});

void PrePhysicsStep(bhkWorld *world);
void PostPhysicsStep(bhkWorld *world);

extern float g_prevVrikOffset;

