#pragma once

#include "RE/havok.h"

#include "skse64_common/Relocation.h"
#include "skse64/PapyrusVM.h"
#include "skse64/GameReferences.h"


typedef bool(*_IsInMenuMode)(VMClassRegistry* registry, UInt32 stackId);
RelocAddr<_IsInMenuMode> IsInMenuMode(0x009F32A0);

typedef void(*_DeleteFunctor)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* objectRefr);
RelocAddr<_DeleteFunctor> DeleteFunctor(0x009CE380);

// untested
typedef char(*_DropObjectFunctor)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* objectRefr, TESForm* akObject, int aiCount);
RelocAddr<_DropObjectFunctor> DropObjectFunctor(0x009CE580);

typedef bool(*_SetMotionTypeFunctor)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* objectRefr, int motionType, bool allowActivate);
RelocAddr<_SetMotionTypeFunctor> SetMotionTypeFunctor(0x009D1FF0);

// untested
typedef char(*_SetPositionFunctor)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* objectRefr, float x, float y, float z);
RelocAddr<_SetPositionFunctor> SetPositionFunctor(0x009D2280);

// untested
typedef char(*_SetAngle)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* objectRefr, float x, float y, float z);
RelocAddr<_SetAngle> SetAngle(0x009D18F0);

typedef float(*_GetHeadingAngle)(VMClassRegistry * registry, uint32_t stackID, TESObjectREFR *source, TESObjectREFR *target);
RelocAddr <_GetHeadingAngle> GetHeadingAngle(0x009CEBB0);

typedef void(*_ApplyHavokImpulse)(VMClassRegistry * registry, uint32_t stackID, TESObjectREFR *target, float afX, float afY, float afZ, float magnitude);
RelocAddr <_ApplyHavokImpulse> ApplyHavokImpulse(0x009CDE90);

typedef bool(*_Activate)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* objectRefr, TESObjectREFR* activator, bool defaultProcessingOnly);
RelocAddr<_Activate> Activate(0x009CD750);

typedef TESObjectREFR* (*_TranslateTo)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* obj, float afX, float afY, float afZ, float afAngleX, float afAngleY, float afAngleZ, float afSpeed, float afMaxRotationSpeed);
RelocAddr<_TranslateTo> TranslateTo(0x009D2570);

typedef TESObjectREFR* (*_StopTranslation)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* obj);
RelocAddr<_StopTranslation> StopTranslation(0x009D3140);

typedef bool(*_Cast)(VMClassRegistry* registry, UInt32 stackId, SpellItem *spell, TESObjectREFR *source, TESObjectREFR *target);
RelocAddr<_Cast> Cast(0x009BB6B0);

typedef bool(*_InterruptCast)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR *caster);
RelocAddr<_InterruptCast> InterruptCast(0x009CEF80);

typedef void(*_EffectShader_Play)(VMClassRegistry* registry, UInt32 stackId, TESEffectShader *shader, TESObjectREFR *target, float duration);
RelocAddr<_EffectShader_Play> EffectShader_Play(0x9BCAF0);

typedef void(*_EffectShader_Stop)(VMClassRegistry* registry, UInt32 stackId, TESEffectShader *shader, TESObjectREFR *target);
RelocAddr<_EffectShader_Stop> EffectShader_Stop(0x9BCC20);

typedef void(*_hkpWorld_CastRay)(ahkpWorld *world, hkpWorldRayCastInput *input, hkpRayHitCollector *collector);
RelocAddr<_hkpWorld_CastRay> hkpWorld_CastRay(0x00AB5B20);

typedef void(*_hkpWorld_LinearCast)(ahkpWorld *world, const hkpCollidable* collA, const hkpLinearCastInput* input, hkpCdPointCollector* castCollector, hkpCdPointCollector* startCollector);
RelocAddr<_hkpWorld_LinearCast> hkpWorld_LinearCast(0x00AB5EC0);