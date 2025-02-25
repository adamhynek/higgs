#pragma once

#include "RE/offsets.h"
#include "RE/misc.h"


RelocPtr<float> g_havokWorldScale(0x15B78F4);
RelocPtr<float> g_inverseHavokWorldScale(0x15ADFE8);

// Alternatively, 0x30008E0 + 0x78
// Even better, (*0x2FC60C0) + 0x78
RelocPtr<bhkSimpleShapePhantom *> g_pickSphere(0x3000958);

RelocPtr<CrosshairPickData *> g_pickData(0x2FC60C0);

RelocPtr<float> g_deltaTime(0x1EC8278);
RelocPtr<float> g_physicsDeltaTime(0x1EC8280);

RelocPtr<UInt32> g_uMaxNumPhysicsStepsPerUpdate(0x1EC8418);
RelocPtr<UInt32> g_uMaxNumPhysicsStepsPerUpdateComplex(0x1EC8430);

RelocPtr<float> g_globalTimeMultiplier(0xC42710);

RelocPtr<float> fMaxTime(0x1EC82B0);
RelocPtr<float> fMaxTimeComplex(0x1EC8448);

RelocPtr<float> g_secondsSinceLastFrame_WorldTime_CheckPaused(0x2FEB794); // like the one below, but is 0 if in menu mode (paused)
RelocPtr<float> g_secondsSinceLastFrame_WorldTime(0x30C3A08); // is multiplied by timeMultiplier
RelocPtr<float> g_secondsSinceLastFrame_GameTime(0x30C3A0C); // is not multiplied by timeMultiplier

RelocPtr<int> g_currentFrameCounter(0x3186C5C);
RelocPtr<int> g_sceneComplexCounter(0x2FEB76C);
RelocPtr<int> g_iShadowUpdateFrameDelay(0x1ED4130);
RelocPtr<int> g_nextShadowUpdateFrameCount(0x3485798);

RelocPtr<ProcessLists *> g_processLists(0x1F831B0);

RelocPtr<BSAudioManager *> g_audioManager(0x30C1D30);

RelocPtr<ShadowSceneNode *> g_shadowSceneNode(0x3423080);

RelocPtr<float> g_minSoundVel(0x1E94F78); // it's an ini setting

RelocPtr<float> g_fMeleeWeaponHavokScale(0x1EAD900); // it's an ini setting

RelocPtr<float> g_fMagicHandTranslateX(0x1EAEA58);
RelocPtr<float> g_fMagicHandTranslateY(0x1EAEA70);
RelocPtr<float> g_fMagicHandTranslateZ(0x1EAEA88);
RelocPtr<float> g_fMagicHandRotateX(0x1EAEAA0);
RelocPtr<float> g_fMagicHandRotateY(0x1EAEAB8);
RelocPtr<float> g_fMagicHandRotateZ(0x1EAEAD0);
RelocPtr<float> g_fMagicHandScale(0x1EAEAE8);

RelocPtr<float> g_fActivateRolloverWandX(0x1EAACF0);
RelocPtr<float> g_fActivateRolloverWandY(0x1EAAD08);
RelocPtr<float> g_fActivateRolloverWandZ(0x1EAAD20);
RelocPtr<float> g_fActivateRolloverWandRotateX(0x1EAAD38);
RelocPtr<float> g_fActivateRolloverWandRotateY(0x1EAAD50);
RelocPtr<float> g_fActivateRolloverWandRotateZ(0x1EAAD68);
RelocPtr<float> g_fActivateRolloverWandScale(0x1EAAD80);

RelocPtr<float> g_fMoveLimitMass(0x1EC84A8);

RelocPtr<DWORD> g_havokMemoryRouterTlsIndex(0x30A8C04);

RelocPtr<hkMemoryAllocator> g_hkContainerHeapAllocator(0x1EB59C8);

RelocPtr<BGSImpactManager> g_impactManager(0x2FEBD60);

// Used by NiCloningProcess...
RelocPtr<UInt64> unk_141E703BC(0x1E703BC);
RelocPtr<UInt64> unk_141E703B8(0x1E703B8);


// Havok / Bethesda havok wrappers
RelocAddr<_hkArrayUtil__reserveMore> hkArrayUtil__reserveMore(0xA02350);
RelocAddr<_hkpWorld_getCurrentTime> hkpWorld_getCurrentTime(0xAB74F0);
RelocAddr<_hkpWorld_CastRay> hkpWorld_CastRay(0x00AB5B20);
RelocAddr<_hkpWorld_LinearCast> hkpWorld_LinearCast(0x00AB5EC0);
RelocAddr<_hkpWorld_GetPenetrations> hkpWorld_GetPenetrations(0x00AB6AA0);
RelocAddr<_hkpWorld_GetClosestPoints> hkpWorld_GetClosestPoints(0xAB62D0);
RelocAddr<_hkpWorld_AddEntity> hkpWorld_AddEntity(0xAB0CB0);
RelocAddr<_hkpWorld_RemoveEntity> hkpWorld_RemoveEntity(0xAB0E50);
RelocAddr<_hkpWorld_addContactListener> hkpWorld_addContactListener(0xAB5580);
RelocAddr<_bhkWorld_addContactListener> bhkWorld_addContactListener(0xDA5C50);
RelocAddr<_hkpEntity_addContactListener> hkpEntity_addContactListener(0xAA6FE0);
RelocAddr<_hkpWorld_addWorldPostSimulationListener> hkpWorld_addWorldPostSimulationListener(0xAB5280);
RelocAddr<_hkpWorld_removeWorldPostSimulationListener> hkpWorld_removeWorldPostSimulationListener(0xAB52E0);
RelocAddr<_hkpWorld_addIslandActivationListener> hkpWorld_addIslandActivationListener(0xAB5100);
RelocAddr<_hkpWorld_removeIslandActivationListener> hkpWorld_removeIslandActivationListener(0xAB5160);
RelocAddr<_hkpWorld_UpdateCollisionFilterOnEntity> hkpWorld_UpdateCollisionFilterOnEntity(0xAB3110);
RelocAddr<_bhkWorld_UpdateCollisionFilterOnEntity> bhkWorld_UpdateCollisionFilterOnEntity(0xDFFE50);
RelocAddr<_ContactListener_PreprocessContactPointEvent> ContactListener_PreprocessContactPointEvent(0xE41AB0); // Checks some shape key stuff and sets disabled on the contact point properties if it wants to
RelocAddr<_hkpSimpleContactConstraintUtil_calculateSeparatingVelocity> hkpSimpleContactConstraintUtil_calculateSeparatingVelocity(0xAAF250);
RelocAddr<_hkpEntity_activate> hkpEntity_activate(0xAA7130);
RelocAddr<_bhkRigidBody_setActivated> bhkRigidBody_setActivated(0xE085D0);
RelocAddr<_bhkRigidBody_GetMaxLinearVelocityMetersPerSecond> bhkRigidBody_GetMaxLinearVelocityMetersPerSecond(0x2B1820);
RelocAddr<_bhkRigidBody_GetMaxAngularVelocity> bhkRigidBody_GetMaxAngularVelocity(0xE0C690);
RelocAddr<_hkpRigidBody_setPositionAndRotation> hkpRigidBody_setPositionAndRotation(0xAA9030);
RelocAddr<_hkpRigidBody_setPosition> hkpRigidBody_setPosition(0xAA8FD0);
RelocAddr<_hkpRigidBody_setRotation> hkpRigidBody_setRotation(0xAA9000);
RelocAddr<_hkpEntity_setTransform> hkpEntity_setTransform(0xAA9060);
RelocAddr<_hkpEntity_getNumConstraints> hkpEntity_getNumConstraints(0xAA73B0);
RelocAddr<_hkpRigidBody_ctor> hkpRigidBody_ctor(0xAA89C0);
RelocAddr<_hkpRigidBodyCinfo_ctor> hkpRigidBodyCinfo_ctor(0xAC5FE0);
RelocAddr<_hkpBoxShape_ctor> hkpBoxShape_ctor(0xA93600);
RelocAddr<_hkpTriggerVolume_ctor> hkpTriggerVolume_ctor(0xAFFCE0);
RelocAddr<_hkpKeyFrameUtility_applyHardKeyFrame> hkpKeyFrameUtility_applyHardKeyFrame(0xAF6DD0);
RelocAddr<_hkpKeyFrameUtility_applyHardKeyFrame> hkpKeyFrameUtility_applyHardKeyFrameAsynchronously(0xAF7100);
RelocAddr<_hkpKeyFrameUtility_applySoftKeyFrame> hkpKeyFrameUtility_applySoftKeyFrame(0xAF6AF0);
RelocAddr<_hkpConstraintInstance_setPriority> hkpConstraintInstance_setPriority(0xAC05B0);
RelocAddr<_hkpMotion_approxTransformAt> hkpMotion_approxTransformAt(0xAAB6E0);
RelocAddr<_bhkCollisionFilter_CompareFilterInfos> bhkCollisionFilter_CompareFilterInfos(0xDD6A80);
RelocAddr<_hkpRigidBody_setMotionType> hkpRigidBody_setMotionType(0xAA9530);
RelocAddr<_bhkRigidBody_setMotionType> bhkRigidBody_setMotionType(0xE08040);
RelocAddr<_bhkRigidBody_MoveToPositionAndRotation> bhkRigidBody_MoveToPositionAndRotation(0xE09210);
RelocAddr<_bhkCollisionObject_SetNodeTransformsFromWorldTransform> bhkCollisionObject_SetNodeTransformsFromWorldTransform(0xE1ACB0);
RelocAddr<_bhkEntity_setPositionAndRotation> bhkEntity_setPositionAndRotation(0xE08350);
RelocAddr<_bhkWorldObject_UpdateCollisionFilter> bhkWorldObject_UpdateCollisionFilter(0xDF88D0);
RelocAddr<_bhkRigidBodyCinfo_ctor> bhkRigidBodyCinfo_ctor(0xE06110);
RelocAddr<_bhkRigidBody_ctor> bhkRigidBody_ctor(0x2AEC80);
RelocAddr<_bhkBoxShape_ctor> bhkBoxShape_ctor(0x2AEB70);
RelocAddr<_hkReferencedObject_addReference> hkReferencedObject_addReference(0xA01280);
RelocAddr<_hkReferencedObject_removeReference> hkReferencedObject_removeReference(0xA01340);
RelocAddr<_hkpCollisionCallbackUtil_requireCollisionCallbackUtil> hkpCollisionCallbackUtil_requireCollisionCallbackUtil(0xAB8700);
RelocAddr<_hkpCollisionCallbackUtil_releaseCollisionCallbackUtil> hkpCollisionCallbackUtil_releaseCollisionCallbackUtil(0xB00C30);
RelocAddr<_hkpWorld_findWorldExtension> hkpWorld_findWorldExtension(0xAB58F0);
RelocAddr<_hkConstraintCinfo_setConstraintData> hkConstraintCinfo_setConstraintData(0xE3E6F0);
RelocAddr<_hkpBallAndSocketConstraintData_setInBodySpace> hkpBallAndSocketConstraintData_setInBodySpace(0xAC1FF0);
RelocAddr<_hkpBallAndSocketConstraintData_setInWorldSpace> hkpBallAndSocketConstraintData_setInWorldSpace(0xAC2010);
RelocAddr<_hkpBallAndSocketConstraintData_ctor> hkpBallAndSocketConstraintData_ctor(0xAC1F60);
RelocAddr<_bhkGroupConstraint_ctor> bhkGroupConstraint_ctor(0xE62630);
RelocAddr<_bhkRigidBody_AddConstraintToArray> bhkRigidBody_AddConstraintToArray(0xE08B10);
RelocAddr<_bhkRigidBody_RemoveConstraintFromArray> bhkRigidBody_RemoveConstraintFromArray(0xE08A70);
RelocAddr<_bhkWorld_AddConstraint> bhkWorld_AddConstraint(0xDFA810);
RelocAddr<_bhkWorld_RemoveConstraint> bhkWorld_RemoveConstraint(0xDFB240);
RelocAddr<_hkpConstraintInstance_isEnabled> hkpConstraintInstance_isEnabled(0xAC06D0);
RelocAddr<_hkVector4_setTransformedInversePos> hkVector4_setTransformedInversePos(0xA05DC0);
RelocAddr<_hkRealTohkUFloat8> hkRealTohkUFloat8(0xA02B10);
RelocAddr<_hkpConstraintData_getConstraintInfoUtil> hkpConstraintData_getConstraintInfoUtil(0xACC490);
RelocAddr<_hkMatrix3_setMul> hkMatrix3_setMul(0xA09A10);
RelocAddr<_hkpPositionConstraintMotor_ctor> hkpPositionConstraintMotor_ctor(0xAD62C0);
RelocAddr<_bhkPositionConstraintMotor_ctor> bhkPositionConstraintMotor_ctor(0xE7A610);
RelocAddr<_bhkCollisionObject_GetRigidBody> bhkCollisionObject_GetRigidBody(0x152EE0);
RelocAddr<_hkpConvexVerticesShape_getOriginalVertices> hkpConvexVerticesShape_getOriginalVertices(0xB98F70);
RelocAddr<_hkpWorld_reintegrateAndRecollideEntities> hkpWorld_reintegrateAndRecollideEntities(0xAB4040);
RelocAddr<_hkpWorldObject_removePropertyMt> hkpWorldObject_removePropertyMt(0xDF8C10);
RelocAddr<_hkpWorldObject_removeProperty> hkpWorldObject_removePropertyImpl(0xAB9B80);
RelocAddr<_hkpWorldObject_setProperty> hkpWorldObject_setProperty(0xAB9C40);
RelocAddr<_hkpWorldObject_hasProperty> hkpWorldObject_hasProperty(0xDF8BD0);
RelocAddr<_hkpCharacterProxy_addCharacterProxyListener> hkpCharacterProxy_addCharacterProxyListener(0xAFA820);

// More havok-related
RelocAddr<_GetHavokWorldFromCell> GetHavokWorldFromCell(0x276A90);
RelocAddr<_GetNodeFromCollidable> GetNodeFromCollidable(0xE01FE0);
RelocAddr<_GetRefFromCollidable> GetRefFromCollidable(0x3B4940);
RelocAddr<_NiAVObject_CollectAllHavokObjects> NiAVObject_CollectAllHavokObjects(0xE03400);

RelocAddr<_CreateDetectionEvent> CreateDetectionEvent(0x656140);
RelocAddr<_ShadowSceneNode_UpdateNodeList> ShadowSceneNode_UpdateNodeList(0x12F89E0);
RelocAddr<_IsInMenuMode> IsInMenuMode(0x009F32A0);
RelocAddr<_ObjectReference_SetActorCause> ObjectReference_SetActorCause(0x9D1830);
RelocAddr<_ObjectReference_Activate> ObjectReference_Activate(0x009CD750);
RelocAddr<_TESObjectREFR_Activate> TESObjectREFR_Activate(0x2A8300);
RelocAddr<_TESObjectREFR_SetScale> TESObjectREFR_SetScale(0x29E3E0);
RelocAddr<_EffectShader_Play> EffectShader_Play(0x9BCAF0);
RelocAddr<_EffectShader_Stop> EffectShader_Stop(0x9BCC20);
RelocAddr<_VisualEffect_Play> VisualEffect_Play(0x9A4E00);
RelocAddr<_VisualEffect_Stop> VisualEffect_Stop(0x9A4F80);
RelocAddr<_Sound_Play> Sound_Play(0x9EF150);
RelocAddr<_BSExtraDataList_RemoveOwnership> BSExtraDataList_RemoveOwnership(0x1309A0);
RelocAddr<_BSExtraDataList_SetOwnerForm> BSExtraDataList_SetOwnerForm(0x11E0C0);
RelocAddr<_TESObjectREFR_SetActorOwner> TESObjectREFR_SetActorOwner(0x9D18C0);
RelocAddr<_NiAVObject_RecalculateWorldTransform> NiAVObject_RecalculateWorldTransform(0xCA7110);
RelocAddr<_ActivatePickRef> ActivatePickRef(0x6CBCE0);
RelocAddr<_TESObjectREFR_GetMass> TESObjectREFR_GetMass(0x9CED20);
RelocAddr<_NiAVObject_GetMass> NiAVObject_GetMass(0x3B5B50);
RelocAddr<_StartGrabObject> StartGrabObject(0x006CC000);
RelocAddr<_TESObjectREFR_SetPosition> TESObjectREFR_SetPosition(0x2A8010);
RelocAddr<_TESObjectREFR_SetRotation> TESObjectREFR_SetRotation(0x2A7C50);
RelocAddr<_NiAVObject_UpdateNode> NiAVObject_UpdateNode(0xC9BC10);
RelocAddr<_GetMaterialType> GetMaterialType(0x2D8B60);
RelocAddr<_BGSImpactDataSet_GetImpactData> BGSImpactDataSet_GetImpactData(0x2D4C00);
RelocAddr<_BSAudioManager_InitSoundData> BSAudioManager_InitSoundData(0xC29D20);
RelocAddr<_SoundData_SetPosition> SoundData_SetPosition(0xC287D0);
RelocAddr<_SoundData_SetNode> SoundData_SetNode(0xC289C0);
RelocAddr<_SoundData_Play> SoundData_Play(0xC283E0);
RelocAddr<_BSExtraList_GetCount> BSExtraList_GetCount(0x123D90);
RelocAddr<_ContainerChanges_GetCount> ContainerChanges_GetCount(0x1F7ED0);
RelocAddr<_EquipManager_EquipEntryData> EquipManager_EquipEntryData(0x641720);
RelocAddr<_TESObjectBOOK_LearnSpell> TESObjectBOOK_LearnSpell(0x23B240);
RelocAddr<_Actor_GetPickupPutdownSound> Actor_GetPickupPutdownSound(0x5D7F90);
RelocAddr<_NiMatrixFromForwardVector> NiMatrixFromForwardVector(0xC4C1E0);
RelocAddr<_NiMatrixToYawPitchRollImpl> NiMatrixToYawPitchRollImpl(0xC9AAA0);
RelocAddr<_NiMatrixToNiQuaternion> NiMatrixToNiQuaternion(0xCB4460);
RelocAddr<_EulerToNiMatrix> EulerToNiMatrix(0xC995A0);
RelocAddr<_UpdateClavicleToTransformHand> UpdateClavicleToTransformHand(0xC4C5A0);
RelocAddr<_NiSkinInstance_UpdateBoneMatrices> NiSkinInstance_UpdateBoneMatrices(0xDC7DC0);
RelocAddr<_NiObject_Clone> NiObject_Clone(0xC978E0);
RelocAddr<_PlayerCharacter_GetOffsetNodeForWeaponIndex> PlayerCharacter_GetOffsetNodeForWeaponIndex(0x6AF100);
RelocAddr<_BSFixedString_Copy> BSFixedString_Copy(0xC6DD50);
RelocAddr<_RefreshActivateButtonArt> RefreshActivateButtonArt(0x53EFE0);
RelocAddr<_Actor_IsInRagdollState> Actor_IsInRagdollState(0x5EBA50);
RelocAddr<_Actor_IsSneaking> Actor_IsSneaking(0x2B19C0);
RelocAddr<_TESRace_IsBeast> TESRace_IsBeast(0x398940);
RelocAddr<_Actor_IsGhost> Actor_IsGhost(0x5DAAE0);
RelocAddr<_Actor_GetMount> Actor_GetMount(0x637A90);
RelocAddr<_Actor_IsBlocking> Actor_IsBlocking(0x611680);
RelocAddr<_IAnimationGraphManagerHolder_SetAnimationVariableFloat> IAnimationGraphManagerHolder_SetAnimationVariableFloat(0x500990);
RelocAddr<_IAnimationGraphManagerHolder_SetAnimationVariableBool> IAnimationGraphManagerHolder_SetAnimationVariableBool(0x500950);

RelocAddr<_HitData_ctor> HitData_ctor(0x76D000);
RelocAddr<_HitData_dtor> HitData_dtor(0x76D0F0);
RelocAddr<_HitData_populate> HitData_populate(0x76D400);
RelocAddr<_HitData_PopulateFromPhysicalHit> HitData_PopulateFromPhysicalHit(0x76DAF0);
RelocAddr<_BSTaskPool_QueueDestructibleDamageTask> BSTaskPool_QueueDestructibleDamageTask(0x5CB4F0);
RelocAddr<_BSTaskPool_QueueDamageObjectTask> BSTaskPool_QueueDamageObjectTask(0x5CB5B0);
RelocAddr<_ActorProcess_GetCurrentlyEquippedWeapon> ActorProcess_GetCurrentlyEquippedWeapon(0x683850);

RelocAddr<_MenuManager_GetMenu> MenuManager_GetMenu(0xF1CE20);
RelocAddr<_NiAVObject_GetRepresentativeCollisionObject> NiAVObject_GetRepresentativeCollisionObject(0xE022C0);

RelocAddr<_PlayRumble> PlayRumble(0xC59440);

// Used by NiCloningProcess...
RelocAddr<_CleanupCloneList> CleanupCloneList1(0x1C8CA0);
RelocAddr<_CleanupCloneList> CleanupCloneList2(0x1C8BE0);

//bhkWorld_Update(0xDFB460);

//BipedAnim_RemoveAllParts(0x1D6530);

// 1st arg: ptr to BipedAnim. 2nd arg: ptr to NiNode
//CreateArmorNode(0x1DB680);
