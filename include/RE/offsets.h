#pragma once

#include "RE/havok.h"
#include "RE/misc.h"
#include "effects.h"

#include "skse64_common/Relocation.h"
#include "skse64/PapyrusVM.h"
#include "skse64/GameExtraData.h"
#include "skse64/GameReferences.h"
#include "skse64/NiNodes.h"
#include "skse64/GameVR.h"
#include "skse64/NiGeometry.h"
#include "skse64/gamethreads.h"
#include "skse64/GameMenus.h"

#include <Physics/Collide/Shape/Convex/Box/hkpBoxShape.h>
#include <Physics/Collide/Shape/Convex/ConvexVertices/hkpConvexVerticesShape.h>
#include <Physics/Dynamics/Constraint/Bilateral/BallAndSocket/hkpBallAndSocketConstraintData.h>
#include <Physics/Dynamics/Constraint/Motor/Position/hkpPositionConstraintMotor.h>
#include <Physics/Utilities/Constraint/Keyframe/hkpKeyFrameUtility.h>
#include <Physics/Utilities/Collide/TriggerVolume/hkpTriggerVolume.h>

// Multiply skyrim coords by this to get havok coords
// It's the number of meters per skyrim unit
extern RelocPtr<float> g_havokWorldScale;
extern RelocPtr<float> g_inverseHavokWorldScale;

// Address of pointer to bhkSimpleShapePhantom that tracks the right hand - more or less
extern RelocPtr<bhkSimpleShapePhantom *> g_pickSphere;

extern RelocPtr<CrosshairPickData *> g_pickData;

extern RelocPtr<float> g_deltaTime;
extern RelocPtr<float> g_physicsDeltaTime;

extern RelocPtr<UInt32> g_uMaxNumPhysicsStepsPerUpdate;
extern RelocPtr<UInt32> g_uMaxNumPhysicsStepsPerUpdateComplex;

extern RelocPtr<float> g_globalTimeMultiplier;

extern RelocPtr<float> fMaxTime;
extern RelocPtr<float> fMaxTimeComplex;

extern RelocPtr<float> g_secondsSinceLastFrame_WorldTime_CheckPaused;
extern RelocPtr<float> g_secondsSinceLastFrame_WorldTime;
extern RelocPtr<float> g_secondsSinceLastFrame_GameTime;

extern RelocPtr<int> g_currentFrameCounter;
extern RelocPtr<int> g_sceneComplexCounter;
extern RelocPtr<int> g_iShadowUpdateFrameDelay;
extern RelocPtr<int> g_nextShadowUpdateFrameCount;

extern RelocPtr<ProcessLists *> g_processLists;

struct BSAudioManager { /* TODO */ };
extern RelocPtr<BSAudioManager *> g_audioManager;

struct ShadowSceneNode : NiNode { /* TODO */ };
extern RelocPtr<ShadowSceneNode *> g_shadowSceneNode;

extern RelocPtr<float> g_minSoundVel;

extern RelocPtr<float> g_fMeleeWeaponHavokScale;

extern RelocPtr<float> g_fMagicHandTranslateX;
extern RelocPtr<float> g_fMagicHandTranslateY;
extern RelocPtr<float> g_fMagicHandTranslateZ;
extern RelocPtr<float> g_fMagicHandRotateX;
extern RelocPtr<float> g_fMagicHandRotateY;
extern RelocPtr<float> g_fMagicHandRotateZ;
extern RelocPtr<float> g_fMagicHandScale;

extern RelocPtr<float> g_fActivateRolloverWandX;
extern RelocPtr<float> g_fActivateRolloverWandY;
extern RelocPtr<float> g_fActivateRolloverWandZ;
extern RelocPtr<float> g_fActivateRolloverWandRotateX;
extern RelocPtr<float> g_fActivateRolloverWandRotateY;
extern RelocPtr<float> g_fActivateRolloverWandRotateZ;
extern RelocPtr<float> g_fActivateRolloverWandScale;

extern RelocPtr<float> g_fMoveLimitMass;

extern RelocPtr<DWORD> g_havokMemoryRouterTlsIndex;
extern RelocPtr<hkMemoryAllocator> g_hkContainerHeapAllocator;


// Havok / Bethesda havok wrappers
typedef void(*_hkArrayUtil__reserveMore)(hkMemoryAllocator *allocator, void *arr, int elemSize);
extern RelocAddr<_hkArrayUtil__reserveMore> hkArrayUtil__reserveMore;

typedef float(*_hkpWorld_getCurrentTime)(hkpWorld *world);
extern RelocAddr<_hkpWorld_getCurrentTime> hkpWorld_getCurrentTime;

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

typedef void* (*_hkpWorld_addContactListener)(hkpWorld *world, hkpContactListener* worldListener);
extern RelocAddr<_hkpWorld_addContactListener> hkpWorld_addContactListener;

typedef void(*_hkpWorld_addWorldPostSimulationListener)(hkpWorld *_this, hkpWorldPostSimulationListener* worldListener);
extern RelocAddr<_hkpWorld_addWorldPostSimulationListener> hkpWorld_addWorldPostSimulationListener;

typedef void(*_hkpWorld_removeWorldPostSimulationListener)(hkpWorld *_this, hkpWorldPostSimulationListener* worldListener);
extern RelocAddr<_hkpWorld_removeWorldPostSimulationListener> hkpWorld_removeWorldPostSimulationListener;

typedef void* (*_hkpWorld_addIslandActivationListener)(hkpWorld *world, hkpIslandActivationListener* worldListener);
extern RelocAddr<_hkpWorld_addIslandActivationListener> hkpWorld_addIslandActivationListener;

typedef void* (*_hkpWorld_removeIslandActivationListener)(hkpWorld *world, hkpIslandActivationListener* worldListener);
extern RelocAddr<_hkpWorld_removeIslandActivationListener> hkpWorld_removeIslandActivationListener;

typedef void* (*_bhkWorld_addContactListener)(bhkWorld *world, hkpContactListener* worldListener);
extern RelocAddr<_bhkWorld_addContactListener> bhkWorld_addContactListener;

typedef void(*_hkpWorld_UpdateCollisionFilterOnEntity)(hkpWorld *world, hkpEntity* entity, hkpUpdateCollisionFilterOnEntityMode updateMode, hkpUpdateCollectionFilterMode updateShapeCollectionFilter);
extern RelocAddr<_hkpWorld_UpdateCollisionFilterOnEntity> hkpWorld_UpdateCollisionFilterOnEntity;

typedef void(*_bhkWorld_UpdateCollisionFilterOnEntity)(bhkWorld *world, hkpEntity* entity);
extern RelocAddr<_bhkWorld_UpdateCollisionFilterOnEntity> bhkWorld_UpdateCollisionFilterOnEntity;

typedef void(*_ContactListener_PreprocessContactPointEvent)(hkpContactListener *listener, const hkpContactPointEvent &evnt);
extern RelocAddr<_ContactListener_PreprocessContactPointEvent> ContactListener_PreprocessContactPointEvent;

typedef float(*_hkpSimpleContactConstraintUtil_calculateSeparatingVelocity)(const hkpRigidBody* bodyA, const hkpRigidBody* bodyB, const hkVector4& centerOfMassInWorldA, const hkVector4& centerOfMassInWorldB, const hkContactPoint* cp);
extern RelocAddr<_hkpSimpleContactConstraintUtil_calculateSeparatingVelocity> hkpSimpleContactConstraintUtil_calculateSeparatingVelocity;

typedef void(*_hkpEntity_activate)(hkpEntity *entity);
extern RelocAddr<_hkpEntity_activate> hkpEntity_activate;

typedef void(*_bhkRigidBody_setActivated)(bhkRigidBody *rigidBody, bool activate);
extern RelocAddr<_bhkRigidBody_setActivated> bhkRigidBody_setActivated;

typedef float(*_bhkRigidBody_GetMaxLinearVelocityMetersPerSecond)(bhkRigidBody *rigidBody);
extern RelocAddr<_bhkRigidBody_GetMaxLinearVelocityMetersPerSecond> bhkRigidBody_GetMaxLinearVelocityMetersPerSecond;

typedef float(*_bhkRigidBody_GetMaxAngularVelocity)(bhkRigidBody *rigidBody);
extern RelocAddr<_bhkRigidBody_GetMaxAngularVelocity> bhkRigidBody_GetMaxAngularVelocity;

typedef void(*_hkpRigidBody_setPositionAndRotation)(hkpEntity *_this, const hkVector4& position, const hkQuaternion& rotation);
extern RelocAddr<_hkpRigidBody_setPositionAndRotation> hkpRigidBody_setPositionAndRotation;

typedef void(*_hkpRigidBody_setPosition)(hkpEntity *_this, const hkVector4& position);
extern RelocAddr<_hkpRigidBody_setPosition> hkpRigidBody_setPosition;

typedef void(*_hkpRigidBody_setRotation)(hkpEntity *_this, const hkQuaternion& rotation);
extern RelocAddr<_hkpRigidBody_setRotation> hkpRigidBody_setRotation;

typedef void(*_hkpEntity_setTransform)(hkpEntity *_this, const hkTransform& transform);
extern RelocAddr<_hkpEntity_setTransform> hkpEntity_setTransform;

typedef int(*_hkpEntity_getNumConstraints)(hkpEntity *_this);
extern RelocAddr<_hkpEntity_getNumConstraints> hkpEntity_getNumConstraints;

typedef void* (*_hkpEntity_addContactListener)(hkpEntity *_this, hkpContactListener* cl);
extern RelocAddr<_hkpEntity_addContactListener> hkpEntity_addContactListener;

typedef void(*_hkpRigidBody_ctor)(hkpRigidBody *_this, hkpRigidBodyCinfo *info);
extern RelocAddr<_hkpRigidBody_ctor> hkpRigidBody_ctor;

typedef void(*_hkpRigidBodyCinfo_ctor)(hkpRigidBodyCinfo *_this);
extern RelocAddr<_hkpRigidBodyCinfo_ctor> hkpRigidBodyCinfo_ctor;

typedef hkpBoxShape* (*_hkpBoxShape_ctor)(hkpBoxShape *_this, const hkVector4& halfExtents, float radius);
extern RelocAddr<_hkpBoxShape_ctor> hkpBoxShape_ctor;

typedef hkpTriggerVolume* (*_hkpTriggerVolume_ctor)(hkpTriggerVolume *_this, hkpRigidBody* triggerBody);
extern RelocAddr<_hkpTriggerVolume_ctor> hkpTriggerVolume_ctor;

typedef void(*_hkpKeyFrameUtility_applyHardKeyFrame)(const hkVector4& nextPosition, const hkQuaternion& nextOrientation, hkReal invDeltaTime, hkpRigidBody* body);
extern RelocAddr<_hkpKeyFrameUtility_applyHardKeyFrame> hkpKeyFrameUtility_applyHardKeyFrame;
extern RelocAddr<_hkpKeyFrameUtility_applyHardKeyFrame> hkpKeyFrameUtility_applyHardKeyFrameAsynchronously;

typedef void(*_hkpKeyFrameUtility_applySoftKeyFrame)(const hkpKeyFrameUtility::KeyFrameInfo& keyFrameInfo, hkpKeyFrameUtility::AccelerationInfo& accelInfo, hkReal deltaTime, hkReal invDeltaTime, hkpRigidBody* body);
extern RelocAddr<_hkpKeyFrameUtility_applySoftKeyFrame> hkpKeyFrameUtility_applySoftKeyFrame;

typedef void(*_hkpConstraintInstance_setPriority)(hkpConstraintInstance *_this, hkpConstraintInstance::ConstraintPriority priority);
extern RelocAddr<_hkpConstraintInstance_setPriority> hkpConstraintInstance_setPriority;

typedef void(*_hkpMotion_approxTransformAt)(hkpMotion *motion, float time, hkTransform& transformOut);
extern RelocAddr<_hkpMotion_approxTransformAt> hkpMotion_approxTransformAt;

typedef bool(*_bhkCollisionFilter_CompareFilterInfos)(bhkCollisionFilter *filter, UInt32 filterInfoA, UInt32 filterInfoB);
extern RelocAddr<_bhkCollisionFilter_CompareFilterInfos> bhkCollisionFilter_CompareFilterInfos;

// newState HAS to be a UInt64, NOT a hkpMotion::MotionType, as hkpMotion::MotionType is a UInt8 but the actual function in the binary expects a UInt64. Fuck.
typedef void (*_hkpRigidBody_setMotionType)(hkpRigidBody *_this, UInt64 newState, hkpEntityActivation preferredActivationState, hkpUpdateCollisionFilterOnEntityMode collisionFilterUpdateMode);
extern RelocAddr<_hkpRigidBody_setMotionType> hkpRigidBody_setMotionType;

typedef void(*_bhkRigidBody_setMotionType)(bhkRigidBody *_this, UInt64 newState, hkpEntityActivation preferredActivationState, hkpUpdateCollisionFilterOnEntityMode collisionFilterUpdateMode);
extern RelocAddr<_bhkRigidBody_setMotionType> bhkRigidBody_setMotionType;

typedef void(*_bhkRigidBody_MoveToPositionAndRotation)(bhkRigidBody *_this, NiPoint3 &pos, NiQuaternion &rot);
extern RelocAddr<_bhkRigidBody_MoveToPositionAndRotation> bhkRigidBody_MoveToPositionAndRotation;

typedef void(*_bhkCollisionObject_SetNodeTransformsFromWorldTransform)(bhkCollisionObject *_this, NiTransform &worldTransform);
extern RelocAddr<_bhkCollisionObject_SetNodeTransformsFromWorldTransform> bhkCollisionObject_SetNodeTransformsFromWorldTransform;

typedef void(*_bhkEntity_setPositionAndRotation)(bhkEntity *_this, const hkVector4& position, const hkQuaternion& rotation);
extern RelocAddr<_bhkEntity_setPositionAndRotation> bhkEntity_setPositionAndRotation;

typedef void(*_bhkWorldObject_UpdateCollisionFilter)(bhkWorldObject *_this);
extern RelocAddr<_bhkWorldObject_UpdateCollisionFilter> bhkWorldObject_UpdateCollisionFilter;

typedef void(*_bhkRigidBodyCinfo_ctor)(bhkRigidBodyCinfo *_this);
extern RelocAddr<_bhkRigidBodyCinfo_ctor> bhkRigidBodyCinfo_ctor;

typedef void(*_bhkRigidBody_ctor)(bhkRigidBody *_this, bhkRigidBodyCinfo *cInfo);
extern RelocAddr<_bhkRigidBody_ctor> bhkRigidBody_ctor;

typedef void(*_bhkBoxShape_ctor)(bhkBoxShape *_this, hkVector4 *halfExtents);
extern RelocAddr<_bhkBoxShape_ctor> bhkBoxShape_ctor;

typedef void(*_hkReferencedObject_addReference)(hkReferencedObject *_this);
extern RelocAddr<_hkReferencedObject_addReference> hkReferencedObject_addReference;

typedef void(*_hkReferencedObject_removeReference)(hkReferencedObject *_this);
extern RelocAddr<_hkReferencedObject_removeReference> hkReferencedObject_removeReference;

typedef bool(*_hkpCollisionCallbackUtil_requireCollisionCallbackUtil)(hkpWorld *world);
extern RelocAddr<_hkpCollisionCallbackUtil_requireCollisionCallbackUtil> hkpCollisionCallbackUtil_requireCollisionCallbackUtil;

typedef bool(*_hkpCollisionCallbackUtil_releaseCollisionCallbackUtil)(hkpWorld *world);
extern RelocAddr<_hkpCollisionCallbackUtil_releaseCollisionCallbackUtil> hkpCollisionCallbackUtil_releaseCollisionCallbackUtil;

typedef hkpWorldExtension * (*_hkpWorld_findWorldExtension)(hkpWorld *world, int id);
extern RelocAddr<_hkpWorld_findWorldExtension> hkpWorld_findWorldExtension;

typedef bool(*_hkConstraintCinfo_setConstraintData)(struct hkConstraintCinfo *_this, hkpConstraintData *data);
extern RelocAddr<_hkConstraintCinfo_setConstraintData> hkConstraintCinfo_setConstraintData;

typedef bool(*_hkpBallAndSocketConstraintData_setInBodySpace)(hkpBallAndSocketConstraintData *_this, const hkVector4 &pivotA, const hkVector4 &pivotB);
extern RelocAddr<_hkpBallAndSocketConstraintData_setInBodySpace> hkpBallAndSocketConstraintData_setInBodySpace;

typedef bool(*_hkpBallAndSocketConstraintData_setInWorldSpace)(hkpBallAndSocketConstraintData *_this, const hkTransform &bodyATransform, const hkTransform &bodyBTransform, const hkVector4 &pivot);
extern RelocAddr<_hkpBallAndSocketConstraintData_setInWorldSpace> hkpBallAndSocketConstraintData_setInWorldSpace;

typedef bool(*_hkpBallAndSocketConstraintData_ctor)(hkpBallAndSocketConstraintData *_this);
extern RelocAddr<_hkpBallAndSocketConstraintData_ctor> hkpBallAndSocketConstraintData_ctor;

typedef bhkGroupConstraint *(*_bhkGroupConstraint_ctor)(bhkGroupConstraint *_this, hkConstraintCinfo *cinfo);
extern RelocAddr<_bhkGroupConstraint_ctor> bhkGroupConstraint_ctor;

typedef void(*_bhkRigidBody_AddConstraintToArray)(bhkRigidBody *_this, bhkConstraint *constraint);
extern RelocAddr<_bhkRigidBody_AddConstraintToArray> bhkRigidBody_AddConstraintToArray;

typedef void(*_bhkRigidBody_RemoveConstraintFromArray)(bhkRigidBody *_this, bhkConstraint *constraint);
extern RelocAddr<_bhkRigidBody_RemoveConstraintFromArray> bhkRigidBody_RemoveConstraintFromArray;

typedef void(*_bhkWorld_AddConstraint)(bhkWorld *_this, hkpConstraintInstance *constraint);
extern RelocAddr<_bhkWorld_AddConstraint> bhkWorld_AddConstraint;

typedef void(*_bhkWorld_RemoveConstraint)(bhkWorld *_this, hkpConstraintInstance *constraint);
extern RelocAddr<_bhkWorld_RemoveConstraint> bhkWorld_RemoveConstraint;

typedef bool *(*_hkpConstraintInstance_isEnabled)(hkpConstraintInstance *_this, bool *enabled);
extern RelocAddr<_hkpConstraintInstance_isEnabled> hkpConstraintInstance_isEnabled;

typedef void(*_hkVector4_setTransformedInversePos)(hkVector4 &_this, const hkTransform &transform, const hkVector4 &pos);
extern RelocAddr<_hkVector4_setTransformedInversePos> hkVector4_setTransformedInversePos;

typedef bool(*_hkRealTohkUFloat8)(hkUFloat8 &out, const hkReal &value);
extern RelocAddr<_hkRealTohkUFloat8> hkRealTohkUFloat8;

typedef bool(*_hkpConstraintData_getConstraintInfoUtil)(const hkpConstraintAtom *atoms, int sizeOfAllAtoms, hkpConstraintData::ConstraintInfo &infoOut);
extern RelocAddr<_hkpConstraintData_getConstraintInfoUtil> hkpConstraintData_getConstraintInfoUtil;

typedef bool(*_hkMatrix3_setMul)(hkMatrix3 &_this, const hkMatrix3 &a, const hkMatrix3 &b);
extern RelocAddr<_hkMatrix3_setMul> hkMatrix3_setMul;

typedef bool(*_hkpPositionConstraintMotor_ctor)(hkpPositionConstraintMotor *_this);
extern RelocAddr<_hkpPositionConstraintMotor_ctor> hkpPositionConstraintMotor_ctor;

typedef bool(*_bhkPositionConstraintMotor_ctor)(hkpPositionConstraintMotor *_this);
extern RelocAddr<_bhkPositionConstraintMotor_ctor> bhkPositionConstraintMotor_ctor;

typedef bhkRigidBody *(*_bhkCollisionObject_GetRigidBody)(bhkCollisionObject *obj);
extern RelocAddr<_bhkCollisionObject_GetRigidBody> bhkCollisionObject_GetRigidBody;

typedef void(*_hkpConvexVerticesShape_getOriginalVertices)(hkpConvexVerticesShape *_this, hkArray<hkVector4> &vertices);
extern RelocAddr<_hkpConvexVerticesShape_getOriginalVertices> hkpConvexVerticesShape_getOriginalVertices;

typedef void(*_hkpWorld_reintegrateAndRecollideEntities)(hkpWorld *world, hkpEntity **entityBatch, int numEntities, UInt32 mode);
extern RelocAddr<_hkpWorld_reintegrateAndRecollideEntities> hkpWorld_reintegrateAndRecollideEntities;

typedef void(*_hkpWorldObject_removePropertyMt)(hkpWorldObject *worldObject, UInt32 key);
extern RelocAddr<_hkpWorldObject_removePropertyMt> hkpWorldObject_removePropertyMt;

typedef UInt64 * (*_hkpWorldObject_removeProperty)(hkpWorldObject *worldObject, UInt64 *retVal, UInt32 key);
extern RelocAddr<_hkpWorldObject_removeProperty> hkpWorldObject_removePropertyImpl;

typedef void(*_hkpWorldObject_setProperty)(hkpWorldObject *worldObject, UInt32 key, UInt64 value);
extern RelocAddr<_hkpWorldObject_setProperty> hkpWorldObject_setProperty;

typedef bool(*_hkpWorldObject_hasProperty)(hkpWorldObject *worldObject, UInt32 key);
extern RelocAddr<_hkpWorldObject_hasProperty> hkpWorldObject_hasProperty;

typedef void(*_hkpCharacterProxy_addCharacterProxyListener)(hkpCharacterProxy *_this, hkpCharacterProxyListener *listener);
extern RelocAddr<_hkpCharacterProxy_addCharacterProxyListener> hkpCharacterProxy_addCharacterProxyListener;

// More havok-related
typedef bhkWorld * (*_GetHavokWorldFromCell)(TESObjectCELL *cell);
extern RelocAddr<_GetHavokWorldFromCell> GetHavokWorldFromCell;

typedef NiAVObject * (*_GetNodeFromCollidable)(const hkpCollidable * a_collidable);
extern RelocAddr<_GetNodeFromCollidable> GetNodeFromCollidable;

typedef TESObjectREFR * (*_GetRefFromCollidable)(const hkpCollidable * a_collidable);
extern RelocAddr<_GetRefFromCollidable> GetRefFromCollidable;

typedef void(*_NiAVObject_CollectAllHavokObjects)(NiAVObject *node, hkArray<hkpRigidBody *> *havokObjects);
extern RelocAddr<_NiAVObject_CollectAllHavokObjects> NiAVObject_CollectAllHavokObjects;


typedef NiTransform * (*_BSVRInterface_GetHandTransform)(BSOpenVR *_this, NiTransform *transformOut, BSVRInterface::BSControllerHand handForOpenVRDeviceIndex, BSVRInterface::BSControllerHand handForBSOpenVRTransform);
extern RelocAddr<_BSVRInterface_GetHandTransform> BSOpenVR_GetHandTransform;

typedef void(*_CreateDetectionEvent)(ActorProcessManager *ownerProcess, Actor *owner, NiPoint3 *position, int soundLevel, TESObjectREFR *source);
extern RelocAddr<_CreateDetectionEvent> CreateDetectionEvent;

typedef void(*_ShadowSceneNode_UpdateNodeList)(ShadowSceneNode *sceneNode, NiAVObject *node, bool isDynamic);
extern RelocAddr<_ShadowSceneNode_UpdateNodeList> ShadowSceneNode_UpdateNodeList;

typedef bool(*_IsInMenuMode)(VMClassRegistry* registry, UInt32 stackId);
extern RelocAddr<_IsInMenuMode> IsInMenuMode;

typedef bool(*_ObjectReference_SetActorCause)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* objectRefr, Actor *actor);
extern RelocAddr<_ObjectReference_SetActorCause> ObjectReference_SetActorCause;

typedef bool(*_ObjectReference_Activate)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR* objectRefr, TESObjectREFR* activator, bool defaultProcessingOnly);
extern RelocAddr<_ObjectReference_Activate> ObjectReference_Activate;

typedef bool(*_TESObjectREFR_Activate)(TESObjectREFR* activatee, TESObjectREFR* activator, UInt32 unk01, UInt32 unk02, UInt32 count, bool defaultProcessingOnly); // unks are 0, 0
extern RelocAddr<_TESObjectREFR_Activate> TESObjectREFR_Activate;

typedef bool(*_TESObjectREFR_SetScale)(TESObjectREFR* refr, float scale);
extern RelocAddr<_TESObjectREFR_SetScale> TESObjectREFR_SetScale;

typedef void(*_EffectShader_Play)(VMClassRegistry* registry, UInt32 stackId, TESEffectShader *shader, TESObjectREFR *target, float duration);
extern RelocAddr<_EffectShader_Play> EffectShader_Play;

typedef void(*_EffectShader_Stop)(VMClassRegistry* registry, UInt32 stackId, TESEffectShader *shader, TESObjectREFR *target);
extern RelocAddr<_EffectShader_Stop> EffectShader_Stop;

typedef void(*_VisualEffect_Play)(VMClassRegistry* registry, UInt32 stackId, BGSReferenceEffect *effect, TESObjectREFR *target, float duration, TESObjectREFR *objToFace);
extern RelocAddr<_VisualEffect_Play> VisualEffect_Play;

typedef void(*_VisualEffect_Stop)(VMClassRegistry* registry, UInt32 stackId, BGSReferenceEffect *effect, TESObjectREFR *target);
extern RelocAddr<_VisualEffect_Stop> VisualEffect_Stop;

typedef UInt32(*_Sound_Play)(VMClassRegistry* registry, UInt32 stackId, BGSSoundDescriptorForm *sound, TESObjectREFR *source);
extern RelocAddr<_Sound_Play> Sound_Play;

typedef void(*_BSExtraDataList_RemoveOwnership)(BaseExtraList *_this);
extern RelocAddr<_BSExtraDataList_RemoveOwnership> BSExtraDataList_RemoveOwnership;

typedef void(*_BSExtraDataList_SetOwnerForm)(BaseExtraList *_this, TESForm *form);
extern RelocAddr<_BSExtraDataList_SetOwnerForm> BSExtraDataList_SetOwnerForm;

typedef void(*_TESObjectREFR_SetActorOwner)(VMClassRegistry* registry, UInt32 stackId, TESObjectREFR *_this, TESForm *owner);
extern RelocAddr<_TESObjectREFR_SetActorOwner> TESObjectREFR_SetActorOwner;

typedef void(*_NiAVObject_RecalculateWorldTransform)(NiAVObject *_this);
extern RelocAddr<_NiAVObject_RecalculateWorldTransform> NiAVObject_RecalculateWorldTransform;

typedef void(*_ActivatePickRef)(PlayerCharacter *player);
extern RelocAddr<_ActivatePickRef> ActivatePickRef;

typedef float(*_TESObjectREFR_GetMass)(float sum, bool firstPerson, TESObjectREFR *obj);
extern RelocAddr<_TESObjectREFR_GetMass> TESObjectREFR_GetMass;

typedef float(*_NiAVObject_GetMass)(NiAVObject *node, float sum);
extern RelocAddr<_NiAVObject_GetMass> NiAVObject_GetMass;

typedef void(*_StartGrabObject)(PlayerCharacter *_this, UInt64 isRightHand);
extern RelocAddr<_StartGrabObject> StartGrabObject;

typedef void(*_TESObjectREFR_SetPosition)(TESObjectREFR *_this, NiPoint3 &newPos);
extern RelocAddr<_TESObjectREFR_SetPosition> TESObjectREFR_SetPosition;

typedef void(*_TESObjectREFR_SetRotation)(TESObjectREFR *_this, NiPoint3 &newRot);
extern RelocAddr<_TESObjectREFR_SetRotation> TESObjectREFR_SetRotation;

typedef void(*_NiAVObject_UpdateNode)(NiAVObject *_this, NiAVObject::ControllerUpdateContext *ctx);
extern RelocAddr<_NiAVObject_UpdateNode> NiAVObject_UpdateNode;

typedef BGSMaterialType * (*_GetMaterialType)(UInt32 materialId); // materialId is gotten from the bhkShape at offset 0x20
extern RelocAddr<_GetMaterialType> GetMaterialType;

typedef BGSImpactData * (*_BGSImpactDataSet_GetImpactData)(BGSImpactDataSet *_this, BGSMaterialType *material);
extern RelocAddr<_BGSImpactDataSet_GetImpactData> BGSImpactDataSet_GetImpactData;

typedef void(*_BSAudioManager_InitSoundData)(BSAudioManager *audioManager, SoundData *soundData, UInt32 formId, int a4); // just pass 16 in a4
extern RelocAddr<_BSAudioManager_InitSoundData> BSAudioManager_InitSoundData;

typedef bool(*_SoundData_SetPosition)(SoundData *soundData, float x, float y, float z);
extern RelocAddr<_SoundData_SetPosition> SoundData_SetPosition;

typedef void(*_SoundData_SetNode)(SoundData *soundData, NiAVObject *node);
extern RelocAddr<_SoundData_SetNode> SoundData_SetNode;

typedef bool(*_SoundData_Play)(SoundData *SoundData);
extern RelocAddr<_SoundData_Play> SoundData_Play;

typedef UInt32(*_BSExtraList_GetCount)(BaseExtraList *extraList);
extern RelocAddr<_BSExtraList_GetCount> BSExtraList_GetCount;

typedef UInt32(*_ContainerChanges_GetCount)(ExtraContainerChanges::Data *_this, TESForm *form);
extern RelocAddr<_ContainerChanges_GetCount> ContainerChanges_GetCount;

typedef void(*_EquipManager_EquipEntryData)(EquipManager *equipManager, Actor *actor, InventoryEntryData *entry, BGSEquipSlot *equipSlot); // equipSlot is null for books
extern RelocAddr<_EquipManager_EquipEntryData> EquipManager_EquipEntryData;

typedef bool(*_TESObjectBOOK_LearnSpell)(TESObjectBOOK *book, Actor *reader);
extern RelocAddr<_TESObjectBOOK_LearnSpell> TESObjectBOOK_LearnSpell;

typedef BGSSoundDescriptorForm * (*_Actor_GetPickupPutdownSound)(Actor *_this, TESBoundObject *object, bool pickup, bool use);
extern RelocAddr<_Actor_GetPickupPutdownSound> Actor_GetPickupPutdownSound;

typedef NiMatrix33 *(*_NiMatrixToYawPitchRollImpl)(NiMatrix33 *mat, float *yaw, float *pitch, float *roll);
extern RelocAddr<_NiMatrixToYawPitchRollImpl> NiMatrixToYawPitchRollImpl;

typedef void(*_NiMatrixToNiQuaternion)(NiQuaternion &quatOut, const NiMatrix33 &matIn);
extern RelocAddr<_NiMatrixToNiQuaternion> NiMatrixToNiQuaternion;

typedef NiMatrix33 *(*_NiMatrixFromForwardVector)(NiMatrix33 *matOut, NiPoint3 *forward, NiPoint3 *world);
extern RelocAddr<_NiMatrixFromForwardVector> NiMatrixFromForwardVector;

typedef NiMatrix33 & (*_EulerToNiMatrix)(NiMatrix33 &matOut, float x, float y, float z);
extern RelocAddr<_EulerToNiMatrix> EulerToNiMatrix;

typedef void (*_UpdateClavicleToTransformHand)(NiAVObject *clavicle, NiAVObject *hand, NiTransform *desiredHandWorldTransform, NiTransform *additionalLocalTransform);
extern RelocAddr<_UpdateClavicleToTransformHand> UpdateClavicleToTransformHand;

typedef void (*_NiSkinInstance_UpdateBoneMatrices)(NiSkinInstance *_this, NiTransform &rootTransform);
extern RelocAddr<_NiSkinInstance_UpdateBoneMatrices> NiSkinInstance_UpdateBoneMatrices;

typedef NiObject * (*_NiObject_Clone)(NiObject *_this, NiCloningProcess *cloningProcess);
extern RelocAddr<_NiObject_Clone> NiObject_Clone;

typedef NiAVObject * (*_PlayerCharacter_GetOffsetNodeForWeaponIndex)(PlayerCharacter *_this, UInt32 isLeft, UInt32 weaponIndex);
extern RelocAddr<_PlayerCharacter_GetOffsetNodeForWeaponIndex> PlayerCharacter_GetOffsetNodeForWeaponIndex;

typedef void(*_BSFixedString_Copy)(BSFixedString *dst, BSFixedString *src);
extern RelocAddr<_BSFixedString_Copy> BSFixedString_Copy;

typedef void(*_RefreshActivateButtonArt)(void *wsActivateRollover);
extern RelocAddr<_RefreshActivateButtonArt> RefreshActivateButtonArt;

typedef bool(*_Actor_IsInRagdollState)(Actor *_this);
extern RelocAddr<_Actor_IsInRagdollState> Actor_IsInRagdollState;

typedef bool(*_Actor_IsSneaking)(Actor *_this);
extern RelocAddr<_Actor_IsSneaking> Actor_IsSneaking;

typedef bool(*_TESRace_IsBeast)(TESRace *_this);
extern RelocAddr<_TESRace_IsBeast> TESRace_IsBeast;

typedef bool(*_Actor_IsGhost)(Actor* _this);
extern RelocAddr<_Actor_IsGhost> Actor_IsGhost;

typedef bool(*_Actor_GetMount)(Actor *_this, NiPointer<Actor> &mount);
extern RelocAddr<_Actor_GetMount> Actor_GetMount;

typedef bool(*_Actor_IsBlocking)(Actor *_this);
extern RelocAddr<_Actor_IsBlocking> Actor_IsBlocking;

typedef bool(*_IAnimationGraphManagerHolder_SetAnimationVariableFloat)(IAnimationGraphManagerHolder *_this, const BSFixedString &variableName, float value);
extern RelocAddr<_IAnimationGraphManagerHolder_SetAnimationVariableFloat> IAnimationGraphManagerHolder_SetAnimationVariableFloat;

typedef bool(*_IAnimationGraphManagerHolder_SetAnimationVariableBool)(IAnimationGraphManagerHolder *_this, const BSFixedString &variableName, bool value);
extern RelocAddr<_IAnimationGraphManagerHolder_SetAnimationVariableBool> IAnimationGraphManagerHolder_SetAnimationVariableBool;

typedef void(*_HitData_ctor)(HitData *_this);
extern RelocAddr<_HitData_ctor> HitData_ctor;

typedef void(*_HitData_dtor)(HitData *_this);
extern RelocAddr<_HitData_dtor> HitData_dtor;

typedef void(*_HitData_populate)(HitData *_this, Actor *src, Actor *target, InventoryEntryData *weapon, bool isOffhand);
extern RelocAddr<_HitData_populate> HitData_populate;

typedef void(*_HitData_PopulateFromPhysicalHit)(HitData *_this, Actor *src, Actor *target, float damage, bhkCharacterController::CollisionEvent &collisionEvent);
extern RelocAddr<_HitData_PopulateFromPhysicalHit> HitData_PopulateFromPhysicalHit;

typedef void(*_BSTaskPool_QueueDestructibleDamageTask)(BSTaskPool *taskPool, TESObjectREFR *target, float damage);
extern RelocAddr<_BSTaskPool_QueueDestructibleDamageTask> BSTaskPool_QueueDestructibleDamageTask;

typedef void(*_BSTaskPool_QueueDamageObjectTask)(BSTaskPool *taskPool, TESObjectREFR *target, float damage, bool isInternalDamage, TESObjectREFR *instigator);
extern RelocAddr<_BSTaskPool_QueueDamageObjectTask> BSTaskPool_QueueDamageObjectTask;

typedef InventoryEntryData *(*_ActorProcess_GetCurrentlyEquippedWeapon)(ActorProcessManager *_this, bool isOffhand);
extern RelocAddr<_ActorProcess_GetCurrentlyEquippedWeapon> ActorProcess_GetCurrentlyEquippedWeapon;

typedef IMenu ** (*_MenuManager_GetMenu)(MenuManager *menuManager, IMenu **menuOut, BSFixedString *menuName);
extern RelocAddr<_MenuManager_GetMenu> MenuManager_GetMenu;

typedef bhkCollisionObject * (*_NiAVObject_GetRepresentativeCollisionObject)(NiAVObject *_this, UInt32 maxChecks);
extern RelocAddr<_NiAVObject_GetRepresentativeCollisionObject> NiAVObject_GetRepresentativeCollisionObject;

typedef void(*_PlayRumble)(UInt32 isRight, float rumbleIntensity, float rumbleDuration);
extern RelocAddr<_PlayRumble> PlayRumble;

