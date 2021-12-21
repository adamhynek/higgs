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

#include <Physics/Collide/Shape/Convex/Box/hkpBoxShape.h>
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

extern RelocPtr<float> g_globalTimeMultiplier;

extern RelocPtr<float> fMaxTime;
extern RelocPtr<float> fMaxTimeComplex;

extern RelocPtr<float> g_secondsSinceLastFrame_WorldTime_CheckPaused;
extern RelocPtr<float> g_secondsSinceLastFrame_WorldTime;
extern RelocPtr<float> g_secondsSinceLastFrame_Unmultiplied;

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


// Havok / Bethesda havok wrappers
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

typedef void* (*_hkpWorld_removeContactListener)(hkpWorld *world, hkpContactListener* worldListener);
extern RelocAddr<_hkpWorld_removeContactListener> hkpWorld_removeContactListener;

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

typedef void(*_hkpEntity_setPositionAndRotation)(hkpEntity *_this, const hkVector4& position, const hkQuaternion& rotation);
extern RelocAddr<_hkpEntity_setPositionAndRotation> hkpEntity_setPositionAndRotation;

typedef void(*_hkpEntity_setTransform)(hkpEntity *_this, const hkTransform& transform);
extern RelocAddr<_hkpEntity_setTransform> hkpEntity_setTransform;

typedef int(*_hkpEntity_getNumConstraints)(hkpEntity *_this);
extern RelocAddr<_hkpEntity_getNumConstraints> hkpEntity_getNumConstraints;

typedef void* (*_hkpEntity_addContactListener)(hkpEntity *_this, hkpContactListener* cl);
extern RelocAddr<_hkpEntity_addContactListener> hkpEntity_addContactListener;

typedef void* (*_hkpEntity_removeContactListener)(hkpEntity *_this, hkpContactListener* cl);
extern RelocAddr<_hkpEntity_removeContactListener> hkpEntity_removeContactListener;

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


// More havok-related
typedef bhkWorld * (*_GetHavokWorldFromCell)(TESObjectCELL *cell);
extern RelocAddr<_GetHavokWorldFromCell> GetHavokWorldFromCell;

typedef NiAVObject * (*_GetNodeFromCollidable)(hkpCollidable * a_collidable);
extern RelocAddr<_GetNodeFromCollidable> GetNodeFromCollidable;

typedef TESObjectREFR * (*_GetRefFromCollidable)(hkpCollidable * a_collidable);
extern RelocAddr<_GetRefFromCollidable> GetRefFromCollidable;


typedef NiTransform * (*_BSVRInterface_GetHandTransform)(BSOpenVR *_this, NiTransform *transformOut, BSVRInterface::BSControllerHand handForOpenVRDeviceIndex, BSVRInterface::BSControllerHand handForBSOpenVRTransform);
extern RelocAddr<_BSVRInterface_GetHandTransform> BSOpenVR_GetHandTransform;

typedef void(*_CreateDetectionEvent)(ActorProcessManager *ownerProcess, Actor *owner, NiPoint3 *position, int soundLevel, TESObjectREFR *source);
extern RelocAddr<_CreateDetectionEvent> CreateDetectionEvent;

typedef void(*_ShadowSceneNode_UpdateNodeList)(ShadowSceneNode *sceneNode, NiAVObject *node, bool useOtherList);
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

typedef void(*_NiMatrixToNiQuaternion)(NiQuaternion &quatOut, const NiMatrix33 &matIn);
extern RelocAddr<_NiMatrixToNiQuaternion> NiMatrixToNiQuaternion;

typedef NiMatrix33 * (*_MatrixFromForwardVector)(NiMatrix33 *matOut, NiPoint3 *forward, NiPoint3 *world);
extern RelocAddr<_MatrixFromForwardVector> MatrixFromForwardVector;

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
