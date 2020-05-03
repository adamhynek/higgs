#pragma once

#include "offsets.h"


RelocPtr<float> HAVOK_WORLD_SCALE_ADDR(0x15B78F4);

// Alternatively, 0x30008E0 + 0x78
// Even better, (*0x2FC60C0) + 0x78
RelocPtr<bhkSimpleShapePhantom *> SPHERE_SHAPE_ADDR(0x3000958);

RelocPtr<UInt32 *> SELECTED_HANDLES(0x2FC60C0);


RelocAddr<_IsInMenuMode> IsInMenuMode(0x009F32A0);

RelocAddr<_SetMotionTypeFunctor> SetMotionTypeFunctor(0x009D1FF0);

RelocAddr <_ApplyHavokImpulse> ApplyHavokImpulse(0x009CDE90);

RelocAddr<_Activate> Activate(0x009CD750);

RelocAddr<_TranslateTo> TranslateTo(0x009D2570);

RelocAddr<_StopTranslation> StopTranslation(0x009D3140);

RelocAddr<_Cast> Cast(0x009BB6B0);

RelocAddr<_InterruptCast> InterruptCast(0x009CEF80);

RelocAddr<_EffectShader_Play> EffectShader_Play(0x9BCAF0);

RelocAddr<_EffectShader_Stop> EffectShader_Stop(0x9BCC20);

RelocAddr<_hkpWorld_CastRay> hkpWorld_CastRay(0x00AB5B20);

RelocAddr<_hkpWorld_LinearCast> hkpWorld_LinearCast(0x00AB5EC0);

RelocAddr<_hkpWorld_GetPenetrations> hkpWorld_GetPenetrations(0x00AB6AA0);

RelocAddr<_hkpEntity_activate> hkpEntity_activate(0xAA7130);

RelocAddr<_FindCollidableRef> FindCollidableRef(0x003B4940);

RelocAddr<_GetWorld> GetWorld(0x276A90);

RelocAddr<_ActivatePickRef> ActivatePickRef(0x6CBCE0);

RelocAddr<_GetMass> GetMass(0x9CED20);
