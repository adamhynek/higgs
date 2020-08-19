#pragma once

#include "RE/havok.h"

#include "skse64_common/Relocation.h"
#include "skse64/PapyrusVM.h"
#include "skse64/GameReferences.h"

#include <Physics/Collide/Shape/Convex/Box/hkpBoxShape.h>
#include <Physics/Utilities/Constraint/Keyframe/hkpKeyFrameUtility.h>


// 0x1F8319C: selected handle as well?
// 0x2FEAC78: TESObjectREFR * to selected object?
struct CrosshairPickData
{
	UInt32 unk00;
	UInt32 leftHandle1; // 04
	UInt32 rightHandle1; // 08
	UInt32 unk0C;
	UInt32 leftHandle2; // 10
	UInt32 rightHandle2; // 14
	UInt32 unk18;
	UInt32 leftHandle3; // 1C
	UInt32 rightHandle3; // 20
	UInt8 unk24[0x50 - 0x24];
	bhkRigidBody *rigidBodies[2]; // 50 - slot depends on hand
	UInt8 unk60[0x78 - 0x60];
	NiPointer<bhkSimpleShapePhantom> sphere; // 78
};
static_assert(offsetof(CrosshairPickData, rigidBodies) == 0x50);
static_assert(offsetof(CrosshairPickData, sphere) == 0x78);


// Multiply skyrim coords by this to get havok coords
// It's the number of meters per skyrim unit
extern RelocPtr<float> g_havokWorldScale;
extern RelocPtr<float> g_inverseHavokWorldScale;

// Address of pointer to bhkSimpleShapePhantom that tracks the right hand - more or less
extern RelocPtr<bhkSimpleShapePhantom *> g_pickSphere;

extern RelocPtr<CrosshairPickData *> g_pickData;

extern RelocPtr<float> g_deltaTime;


typedef bool(*_IsInMenuMode)(VMClassRegistry* registry, UInt32 stackId);
extern RelocAddr<_IsInMenuMode> IsInMenuMode;

typedef bool(*_Activate)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* objectRefr, TESObjectREFR* activator, bool defaultProcessingOnly);
extern RelocAddr<_Activate> Activate;

typedef void(*_EffectShader_Play)(VMClassRegistry* registry, UInt32 stackId, TESEffectShader *shader, TESObjectREFR *target, float duration);
extern RelocAddr<_EffectShader_Play> EffectShader_Play;

typedef void(*_EffectShader_Stop)(VMClassRegistry* registry, UInt32 stackId, TESEffectShader *shader, TESObjectREFR *target);
extern RelocAddr<_EffectShader_Stop> EffectShader_Stop;

typedef void(*_VisualEffect_Play)(VMClassRegistry* registry, UInt32 stackId, BGSReferenceEffect *effect, TESObjectREFR *target, float duration, TESObjectREFR *objToFace);
extern RelocAddr<_VisualEffect_Play> VisualEffect_Play;

typedef void(*_VisualEffect_Stop)(VMClassRegistry* registry, UInt32 stackId, BGSReferenceEffect *effect, TESObjectREFR *target);
extern RelocAddr<_VisualEffect_Stop> VisualEffect_Stop;

typedef void(*_BSExtraDataList_RemoveOwnership)(BaseExtraList *_this);
extern RelocAddr<_BSExtraDataList_RemoveOwnership> BSExtraDataList_RemoveOwnership;

typedef void(*_BSExtraDataList_SetOwnerForm)(BaseExtraList *_this, TESForm *form);
extern RelocAddr<_BSExtraDataList_SetOwnerForm> BSExtraDataList_SetOwnerForm;

typedef void(*_TESObjectREFR_SetActorOwner)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR *_this, TESForm *owner);
extern RelocAddr<_TESObjectREFR_SetActorOwner> TESObjectREFR_SetActorOwner;

typedef void(*_hkpWorld_CastRay)(hkpWorld *world, hkpWorldRayCastInput *input, hkpRayHitCollector *collector);
extern RelocAddr<_hkpWorld_CastRay> hkpWorld_CastRay;

typedef void(*_hkpWorld_LinearCast)(hkpWorld *world, const hkpCollidable* collA, const hkpLinearCastInput* input, hkpCdPointCollector* castCollector, hkpCdPointCollector* startCollector);
extern RelocAddr<_hkpWorld_LinearCast> hkpWorld_LinearCast;

typedef void(*_hkpWorld_GetPenetrations)(hkpWorld *world, const hkpCollidable* collA, const hkpCollisionInput* input, hkpCdBodyPairCollector* collector);
extern RelocAddr<_hkpWorld_GetPenetrations> hkpWorld_GetPenetrations;

typedef void(*_hkpWorld_GetClosestPoints)(hkpWorld *world, const hkpCollidable* collA, const hkpCollisionInput* input, hkpCdPointCollector* collector);
extern RelocAddr<_hkpWorld_GetClosestPoints> hkpWorld_GetClosestPoints;

typedef hkpEntity* (*_hkpWorld_AddEntity)(hkpWorld *world, hkpEntity* entity, hkpEntityActivation initialActivationState);
extern RelocAddr<_hkpWorld_AddEntity> hkpWorld_AddEntity;

typedef hkpEntity* (*_hkpWorld_RemoveEntity)(hkpWorld *world, hkBool *ret, hkpEntity* entity);
extern RelocAddr<_hkpWorld_RemoveEntity> hkpWorld_RemoveEntity;

typedef void(*_hkpWorld_UpdateCollisionFilterOnEntity)(hkpWorld *world, hkpEntity* entity, hkpUpdateCollisionFilterOnEntityMode updateMode, hkpUpdateCollectionFilterMode updateShapeCollectionFilter);
extern RelocAddr<_hkpWorld_UpdateCollisionFilterOnEntity> hkpWorld_UpdateCollisionFilterOnEntity;

typedef void(*_bhkWorld_UpdateCollisionFilterOnEntity)(bhkWorld *world, hkpEntity* entity);
extern RelocAddr<_bhkWorld_UpdateCollisionFilterOnEntity> bhkWorld_UpdateCollisionFilterOnEntity;

typedef void(*_hkpEntity_activate)(hkpEntity *entity);
extern RelocAddr<_hkpEntity_activate> hkpEntity_activate;

typedef void(*_bhkRigidBody_setActivated)(bhkRigidBody *rigidBody, bool activate);
extern RelocAddr<_bhkRigidBody_setActivated> bhkRigidBody_setActivated;

typedef void(*_hkpEntity_setPositionAndRotation)(hkpEntity *_this, const hkVector4& position, const hkVector4& rotation); // rotation is hkQuaternion
extern RelocAddr<_hkpEntity_setPositionAndRotation> hkpEntity_setPositionAndRotation;

typedef void(*_hkpEntity_setTransform)(hkpEntity *_this, const hkTransform& transform);
extern RelocAddr<_hkpEntity_setTransform> hkpEntity_setTransform;

typedef void(*_hkpRigidBody_ctor)(hkpRigidBody *_this, hkpRigidBodyCinfo *info);
extern RelocAddr<_hkpRigidBody_ctor> hkpRigidBody_ctor;

typedef void(*_hkpRigidBodyCinfo_ctor)(hkpRigidBodyCinfo *_this);
extern RelocAddr<_hkpRigidBodyCinfo_ctor> hkpRigidBodyCinfo_ctor;

typedef hkpBoxShape* (*_hkpBoxShape_ctor)(hkpBoxShape *_this, const hkVector4& halfExtents, float radius);
extern RelocAddr<_hkpBoxShape_ctor> hkpBoxShape_ctor;

typedef void(*_hkpKeyFrameUtility_applyHardKeyFrame)(const hkVector4& nextPosition, const hkQuaternion& nextOrientation, hkReal invDeltaTime, hkpRigidBody* body);
extern RelocAddr<_hkpKeyFrameUtility_applyHardKeyFrame> hkpKeyFrameUtility_applyHardKeyFrame;
extern RelocAddr<_hkpKeyFrameUtility_applyHardKeyFrame> hkpKeyFrameUtility_applyHardKeyFrameAsynchronously;

typedef void(*_hkpKeyFrameUtility_applySoftKeyFrame)(const hkpKeyFrameUtility::KeyFrameInfo& keyFrameInfo, hkpKeyFrameUtility::AccelerationInfo& accelInfo, hkReal deltaTime, hkReal invDeltaTime, hkpRigidBody* body);
extern RelocAddr<_hkpKeyFrameUtility_applySoftKeyFrame> hkpKeyFrameUtility_applySoftKeyFrame;

typedef void(*_hkpConstraintInstance_setPriority)(hkpConstraintInstance *_this, hkpConstraintInstance::ConstraintPriority priority);
extern RelocAddr<_hkpConstraintInstance_setPriority> hkpConstraintInstance_setPriority;

// MotionType HAS to be a UInt64, NOT a hkpMotion::MotionType, as hkpMotion::MotionType is a UInt8 but the actual function in the binary expects a UInt64. Fuck.
typedef void (*_hkpRigidBody_setMotionType)(hkpRigidBody *_this, UInt64 newState, hkpEntityActivation preferredActivationState, hkpUpdateCollisionFilterOnEntityMode collisionFilterUpdateMode);
extern RelocAddr<_hkpRigidBody_setMotionType> hkpRigidBody_setMotionType;

typedef void(*_bhkRigidBody_setMotionType)(bhkRigidBody *_this, UInt64 newState, hkpEntityActivation preferredActivationState, hkpUpdateCollisionFilterOnEntityMode collisionFilterUpdateMode);
extern RelocAddr<_bhkRigidBody_setMotionType> bhkRigidBody_setMotionType;

typedef void(*_bhkEntity_setPositionAndRotation)(bhkEntity *_this, const hkVector4& position, const hkVector4& rotation); // rotation is hkQuaternion
extern RelocAddr<_bhkEntity_setPositionAndRotation> bhkEntity_setPositionAndRotation;

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

typedef void(*_StartGrabObject)(PlayerCharacter *_this, UInt64 isRightHand);
extern RelocAddr<_StartGrabObject> StartGrabObject;

typedef void(*_AddRemoveConstraintFunctor)(__int64 a1, void *a2);
extern RelocAddr<_AddRemoveConstraintFunctor> AddRemoveConstraintFunctor;

typedef void(*_AddHavokBallAndSocketConstraint)(VMClassRegistry *registry, UInt32 stackId, UInt64 unk, TESObjectREFR *refA, const char **refANode, TESObjectREFR *refB, const char **refBNode,
	float afRefALocalOffsetX, float afRefALocalOffsetY, float afRefALocalOffsetZ, float afRefBLocalOffsetX, float afRefBLocalOffsetY, float afRefBLocalOffsetZ);
extern RelocAddr<_AddHavokBallAndSocketConstraint> AddHavokBallAndSocketConstraint;

typedef void(*_RemoveHavokConstraints)(VMClassRegistry *registry, UInt32 stackId, UInt64 unk, TESObjectREFR *refA, const char **refANode, TESObjectREFR *refB, const char **refBNode);
extern RelocAddr<_RemoveHavokConstraints> RemoveHavokConstraints;

typedef void(*_TESObjectREFR_SetPosition)(TESObjectREFR *_this, NiPoint3 &newPos);
extern RelocAddr<_TESObjectREFR_SetPosition> TESObjectREFR_SetPosition;

typedef void(*_TESObjectREFR_SetRotation)(TESObjectREFR *_this, NiPoint3 &newRot);
extern RelocAddr<_TESObjectREFR_SetRotation> TESObjectREFR_SetRotation;

typedef void(*_NiAVObject_UpdateObjectUpwards)(NiAVObject *_this, NiAVObject::ControllerUpdateContext *ctx);
extern RelocAddr<_NiAVObject_UpdateObjectUpwards> NiAVObject_UpdateObjectUpwards;

typedef void(*_hkReferencedObject_addReference)(hkReferencedObject *_this);
extern RelocAddr<_hkReferencedObject_addReference> hkReferencedObject_addReference;

typedef void(*_hkReferencedObject_removeReference)(hkReferencedObject *_this);
extern RelocAddr<_hkReferencedObject_removeReference> hkReferencedObject_removeReference;
