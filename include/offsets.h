#pragma once

#include "RE/havok.h"

#include "skse64_common/Relocation.h"
#include "skse64/PapyrusVM.h"
#include "skse64/GameReferences.h"


// Multiply skyrim coords by this to get havok coords
// It's the number of meters per skyrim unit
extern RelocPtr<float> HAVOK_WORLD_SCALE_ADDR;

// Address of pointer to bhkSimpleShapePhantom that tracks the right hand - more or less
extern RelocPtr<bhkSimpleShapePhantom *> SPHERE_SHAPE_ADDR;

extern RelocPtr<UInt32 *> SELECTED_HANDLES;


typedef bool(*_IsInMenuMode)(VMClassRegistry* registry, UInt32 stackId);
extern RelocAddr<_IsInMenuMode> IsInMenuMode;

typedef bool(*_SetMotionTypeFunctor)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* objectRefr, int motionType, bool allowActivate);
extern RelocAddr<_SetMotionTypeFunctor> SetMotionTypeFunctor;

typedef void(*_ApplyHavokImpulse)(VMClassRegistry * registry, uint32_t stackID, TESObjectREFR *target, float afX, float afY, float afZ, float magnitude);
extern RelocAddr <_ApplyHavokImpulse> ApplyHavokImpulse;

typedef bool(*_Activate)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* objectRefr, TESObjectREFR* activator, bool defaultProcessingOnly);
extern RelocAddr<_Activate> Activate;

typedef TESObjectREFR* (*_TranslateTo)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* obj, float afX, float afY, float afZ, float afAngleX, float afAngleY, float afAngleZ, float afSpeed, float afMaxRotationSpeed);
extern RelocAddr<_TranslateTo> TranslateTo;

typedef TESObjectREFR* (*_StopTranslation)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* obj);
extern RelocAddr<_StopTranslation> StopTranslation;

typedef bool(*_Cast)(VMClassRegistry* registry, UInt32 stackId, SpellItem *spell, TESObjectREFR *source, TESObjectREFR *target);
extern RelocAddr<_Cast> Cast;

typedef bool(*_InterruptCast)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR *caster);
extern RelocAddr<_InterruptCast> InterruptCast;

typedef void(*_EffectShader_Play)(VMClassRegistry* registry, UInt32 stackId, TESEffectShader *shader, TESObjectREFR *target, float duration);
extern RelocAddr<_EffectShader_Play> EffectShader_Play;

typedef void(*_EffectShader_Stop)(VMClassRegistry* registry, UInt32 stackId, TESEffectShader *shader, TESObjectREFR *target);
extern RelocAddr<_EffectShader_Stop> EffectShader_Stop;

typedef void(*_hkpWorld_CastRay)(ahkpWorld *world, hkpWorldRayCastInput *input, hkpRayHitCollector *collector);
extern RelocAddr<_hkpWorld_CastRay> hkpWorld_CastRay;

typedef void(*_hkpWorld_LinearCast)(ahkpWorld *world, const hkpCollidable* collA, const hkpLinearCastInput* input, hkpCdPointCollector* castCollector, hkpCdPointCollector* startCollector);
extern RelocAddr<_hkpWorld_LinearCast> hkpWorld_LinearCast;

typedef void(*_hkpWorld_GetPenetrations)(ahkpWorld *world, const hkpCollidable* collA, const hkpCollisionInput* input, hkpCdBodyPairCollector* collector);
extern RelocAddr<_hkpWorld_GetPenetrations> hkpWorld_GetPenetrations;

typedef void(*_hkpEntity_activate)(hkpEntity *entity);
extern RelocAddr<_hkpEntity_activate> hkpEntity_activate;

typedef TESObjectREFR* (*_FindCollidableRef)(hkpCollidable * a_collidable);
extern RelocAddr<_FindCollidableRef> FindCollidableRef;

typedef NiAVObject* (*_FindCollidableNode)(hkpCollidable * a_collidable);
extern RelocAddr<_FindCollidableNode> FindCollidableNode;

typedef bhkWorld* (*_GetWorld)(TESObjectCELL *cell);
extern RelocAddr<_GetWorld> GetWorld;

typedef void(*_ActivatePickRef)(PlayerCharacter *player);
extern RelocAddr<_ActivatePickRef> ActivatePickRef;

typedef float(*_GetMass)(float sum, bool firstPerson, TESObjectREFR *obj);
extern RelocAddr<_GetMass> GetMass;

typedef void(*_AddRemoveConstraintFunctor)(__int64 a1, void *a2);
extern RelocAddr<_AddRemoveConstraintFunctor> AddRemoveConstraintFunctor;
