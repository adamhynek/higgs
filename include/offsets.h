#pragma once

#include "RE/havok.h"

#include "skse64_common/Relocation.h"
#include "skse64/PapyrusVM.h"
#include "skse64/GameReferences.h"


// Multiply skyrim coords by this to get havok coords
// It's the number of meters per skyrim unit
RelocPtr<float> HAVOK_WORLD_SCALE_ADDR(0x15B78F4);

// Alternatively, 0x30008E0 + 0x78
// Even better, (*0x2FC60C0) + 0x78
// Address of pointer to bhkSimpleShapePhantom that tracks the right hand - more or less
RelocPtr<bhkSimpleShapePhantom *> SPHERE_SHAPE_ADDR(0x3000958);

RelocPtr<UInt32 *> SELECTED_HANDLES(0x2FC60C0);


typedef bool(*_IsInMenuMode)(VMClassRegistry* registry, UInt32 stackId);
RelocAddr<_IsInMenuMode> IsInMenuMode(0x009F32A0);

typedef bool(*_SetMotionTypeFunctor)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* objectRefr, int motionType, bool allowActivate);
RelocAddr<_SetMotionTypeFunctor> SetMotionTypeFunctor(0x009D1FF0);

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

typedef void(*_hkpWorld_GetPenetrations)(ahkpWorld *world, const hkpCollidable* collA, const hkpCollisionInput* input, hkpCdBodyPairCollector* collector);
RelocAddr<_hkpWorld_GetPenetrations> hkpWorld_GetPenetrations(0x00AB6AA0);

typedef TESObjectREFR* (*_FindCollidableRef)(hkpCollidable * a_collidable);
RelocAddr<_FindCollidableRef> FindCollidableRef(0x003B4940);

typedef bhkWorld* (*_GetWorld)(TESObjectCELL *cell);
RelocAddr<_GetWorld> GetWorld(0x276A90);

typedef void(*_ActivatePickRef)(PlayerCharacter *player);
RelocAddr<_ActivatePickRef> ActivatePickRef(0x6CBCE0);
