#pragma once

#include "offsets.h"


RelocPtr<float> HAVOK_WORLD_SCALE_ADDR(0x15B78F4);

// Alternatively, 0x30008E0 + 0x78
// Even better, (*0x2FC60C0) + 0x78
RelocPtr<bhkSimpleShapePhantom *> SPHERE_SHAPE_ADDR(0x3000958);

RelocPtr<UInt32 *> SELECTED_HANDLES(0x2FC60C0);

RelocPtr<float> g_deltaTime(0x1EC8278);


RelocAddr<_IsInMenuMode> IsInMenuMode(0x009F32A0);

RelocAddr<_SetMotionTypeFunctor> SetMotionTypeFunctor(0x009D1FF0);

RelocAddr <_ApplyHavokImpulse> ApplyHavokImpulse(0x009CDE90);

RelocAddr<_Activate> Activate(0x009CD750);

RelocAddr<_SetPosition> SetPosition(0x9D2280);

RelocAddr<_TranslateTo> TranslateTo(0x009D2570);

RelocAddr<_StopTranslation> StopTranslation(0x009D3140);

RelocAddr<_Cast> Cast(0x009BB6B0);

RelocAddr<_InterruptCast> InterruptCast(0x009CEF80);

RelocAddr<_EffectShader_Play> EffectShader_Play(0x9BCAF0);

RelocAddr<_EffectShader_Stop> EffectShader_Stop(0x9BCC20);

RelocAddr<_hkpWorld_CastRay> hkpWorld_CastRay(0x00AB5B20);

RelocAddr<_hkpWorld_LinearCast> hkpWorld_LinearCast(0x00AB5EC0);

RelocAddr<_hkpWorld_GetPenetrations> hkpWorld_GetPenetrations(0x00AB6AA0);

RelocAddr<_hkpWorld_GetClosestPoints> hkpWorld_GetClosestPoints(0xAB62D0);

RelocAddr<_hkpWorld_AddEntity> hkpWorld_AddEntity(0xAB0CB0);

RelocAddr<_hkpWorld_UpdateCollisionFilterOnEntity> hkpWorld_UpdateCollisionFilterOnEntity(0xAB3110);

RelocAddr<_hkpEntity_activate> hkpEntity_activate(0xAA7130);

RelocAddr<_hkpEntity_setPositionAndRotation> hkpEntity_setPositionAndRotation(0xAA9030);

RelocAddr<_hkpEntity_setTransform> hkpEntity_setTransform(0xAA9060);

RelocAddr<_hkpRigidBody_ctor> hkpRigidBody_ctor(0xAA89C0);

RelocAddr<_hkpRigidBodyCinfo_ctor> hkpRigidBodyCinfo_ctor(0xAC5FE0); // just sets all the fields to defaults

RelocAddr<_hkpBoxShape_ctor> hkpBoxShape_ctor(0xA93600);

RelocAddr<_hkpKeyFrameUtility_applyHardKeyFrame> hkpKeyFrameUtility_applyHardKeyFrame(0xAF6DD0);
RelocAddr<_hkpKeyFrameUtility_applyHardKeyFrame> hkpKeyFrameUtility_applyHardKeyFrameAsynchronously(0xAF7100);

RelocAddr<_hkpKeyFrameUtility_applySoftKeyFrame> hkpKeyFrameUtility_applySoftKeyFrame(0xAF6AF0);

RelocAddr<_hkpRigidBody_setMotionType> hkpRigidBody_setMotionType(0xAA9530); // bhkRigidBody_setMotionType is at E08040

RelocAddr<_bhkRigidBody_setMotionType> bhkRigidBody_setMotionType(0xE08040);

RelocAddr<_bhkEntity_setPositionAndRotation> bhkEntity_setPositionAndRotation(0xE08350);

RelocAddr<_FindCollidableRef> FindCollidableRef(0x003B4940);

RelocAddr<_FindCollidableNode> FindCollidableNode(0xE01FE0);

RelocAddr<_GetWorld> GetWorld(0x276A90);

RelocAddr<_ActivatePickRef> ActivatePickRef(0x6CBCE0);

RelocAddr<_GetMass> GetMass(0x9CED20);

RelocAddr<_StartGrabObject> StartGrabObject(0x006CC000);

RelocAddr<_AddRemoveConstraintFunctor> AddRemoveConstraintFunctor(0x9AB270);

RelocAddr<_AddHavokBallAndSocketConstraint> AddHavokBallAndSocketConstraint(0x9ABAB0);

RelocAddr<_RemoveHavokConstraints> RemoveHavokConstraints(0x9ABD40);

RelocAddr<_TESObjectREFR_SetPosition> TESObjectREFR_SetPosition(0x2A8010);

RelocAddr<_TESObjectREFR_SetRotation> TESObjectREFR_SetRotation(0x2A7C50);

RelocAddr<_NiAVObject_UpdateObjectUpwards> NiAVObject_UpdateObjectUpwards(0xC9BC10);

//BipedAnim_RemoveAllParts(0x1D6530);

// 1st arg: ptr to BipedAnim. 2nd arg: ptr to NiNode
//CreateArmorNode(0x1DB680);


//115111
