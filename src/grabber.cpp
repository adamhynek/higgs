#include <numeric>

#include "skse64/GameRTTI.h"
#include "skse64/PapyrusActor.h"
#include "skse64/NiGeometry.h"
#include "skse64/GameExtraData.h"

#include "grabber.h"
#include "offsets.h"
#include "utils.h"
#include "config.h"
#include "menu_checker.h"
#include "shaders.h"

#include <Physics/Collide/Query/CastUtil/hkpLinearCastInput.h>
#include <Physics/Collide/Query/CastUtil/hkpWorldRayCastInput.h>
#include <Physics/Collide/Agent3/Machine/Nn/hkpLinkedCollidable.h>


BSFixedString hmdNodeStr("HmdNode");

// Gets callbacks from havok linear cast
CdPointCollector cdPointCollector;
hkpLinearCastInput linearCastInput;
RayHitCollector rayHitCollector;
AllRayHitCollector allRayHitCollector;
CdBodyPairCollector pairCollector;
// 'ItemPicker' collision layer; player collision group
// Why ItemPicker? It ignores stuff like weapons the player is holding
//static hkpWorldRayCastInput rayCastInput(0x02420028);
hkpWorldRayCastInput rayCastInput;


// Copied from PapyrusActor.cpp
class MatchByForm : public FormMatcher
{
	TESForm * m_form;
public:
	MatchByForm(TESForm * form) : m_form(form) {}

	bool Matches(TESForm* pForm) const { return m_form == pForm; }
};


void Grabber::Select(TESObjectREFR *obj)
{
	selectedObject.handle = GetOrCreateRefrHandle(obj);

	selectedObject.isImpactedProjectile = false;
	auto baseForm = obj->baseForm;
	if (baseForm && baseForm->formType == kFormType_Projectile) {
		auto impactData = *(void **)((UInt64)obj + 0x98);
		if (impactData) {
			// If the projectile has impact data, then it has well, impacted something
			selectedObject.isImpactedProjectile = true;
		}
	}

	selectedObject.isActor = false;
	auto actor = DYNAMIC_CAST(obj, TESObjectREFR, Actor);
	if (actor) {
		selectedObject.isActor = true;
	}
}


void Grabber::Deselect()
{
	selectedObject.handle = *g_invalidRefHandle;
	selectedObject.collidable = nullptr;
	selectedObject.rigidBody = nullptr;
	selectedObject.shaderNode = nullptr;
	selectedObject.hitNode = nullptr;
	selectedObject.hitForm = nullptr;

	state = IDLE;
}


UInt32 priorities[] = {
	3, // head
	2, // hair
	9, // body
	5, // hands
	6, // forearms
	1, // amulet
	4, // ring
	7, // feet
	8, // calves
	10, // shield
	13, // tail - seems to be used by cloaks
	11, // longhair
	0, // circlet
	12, // ears
	84, // 14
	85, // 15
	14, // 16 - seems to be used by cloaks as well
	86, // 17
	87, // 18
	88, // 19
	89, // decapitatehead
	90, // decapitate
	91, // 22
	92, // 23
	93, // 24
	94, // 25
	95, // 26
	96, // 27
	97, // 28
	98, // 29
	99, // 30
	100  // fx01
};
bool CompareBipedIndices(int i1, int i2)
{
	// Return true if second index beats first index
	return priorities[i2] < priorities[i1];
}


void Grabber::PoseUpdate(const Grabber &other, bool allowGrab, NiNode *playerWorldNode)
{
	PlayerCharacter *player = *g_thePlayer;
	if (!player || !player->loadedState || !player->loadedState->node)
		return;

	TESObjectCELL* cell = player->parentCell;
	if (!cell)
		return;

	NiAVObject *handNode = player->GetNiRootNode(1)->GetObjectByName(&handNodeName.data);
	if (!handNode)
		return;

	NiPoint3 handPos = handNode->m_worldTransform.pos;

	NiPoint3 castDirection = VectorNormalized(handNode->m_worldTransform.rot * Config::options.handAdjust);

	NiAVObject *hmdNode = playerWorldNode->GetObjectByName(&hmdNodeStr.data);
	if (!hmdNode)
		return;

	NiPoint3 hmdPos = hmdNode->m_worldTransform.pos;

	NiPoint3 hmdForward = { hmdNode->m_worldTransform.rot.data[0][1], hmdNode->m_worldTransform.rot.data[1][1], hmdNode->m_worldTransform.rot.data[2][1] };

	NiAVObject *wandNode = playerWorldNode->GetObjectByName(&wandNodeName.data);
	if (!wandNode)
		return;

	static BSFixedString comName("NPC COM [COM ]");
	NiAVObject *comNode = player->GetNiRootNode(0)->GetObjectByName(&comName.data);
	if (!comNode) {
		_MESSAGE("No COM [COM ] node on player");
		return;
	}
	if (!comNode->unk040) {
		_MESSAGE("COM node has no collision object");
		return;
	}

	UInt16 playerCollisionGroup = ((bhkCollisionObject *)comNode->unk040)->body->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo >> 16;

	bhkWorld *world = GetWorld(cell);
	if (!world) {
		_MESSAGE("Could not get havok world from player cell");
		return;
	}

	float havokWorldScale = *HAVOK_WORLD_SCALE_ADDR;

	if (world->world != handCollBody->m_world) {
		// Create our own layer in the first ununsed vanilla layer (56)
		bhkCollisionFilter *worldFilter = (bhkCollisionFilter *)world->world->m_collisionFilter;
		UInt64 bitfield = 0x00053343561B7EFF; // copy of L_WEAPON layer bitfield - TODO: Actually just copy the L_WEAPON layer bitfield?
		bitfield |= ((UInt64)1 << 56); // collide with ourselves
		bitfield &= ~((UInt64)1 << 0x1e); // remove collision with character controllers
		worldFilter->layerBitfields[56] = bitfield;
		worldFilter->layerNames[56] = "L_HANDCOLLISION";
		// Set whether other layers should collide with our new layer
		for (int i = 0; i < 56; i++) {
			if ((bitfield >> i) & 1) {
				worldFilter->layerBitfields[i] |= ((UInt64)1 << 56);
			}
		}

		// Add collision object for the hand
		hkpBoxShape_ctor(handCollShape, { 0.05, 0.015, 0.075, 0 }, 0);
		hkpRigidBodyCinfo_ctor(handCollCInfo); // initialize with defaults
		handCollCInfo->m_shape = handCollShape;
		handCollCInfo->m_collisionFilterInfo = ((UInt32)playerCollisionGroup << 16) | 56; // player group, our custom layer
		handCollCInfo->m_collisionFilterInfo |= (1 << 15); // set bit 15 to collide with same group that also has bit 15
		handCollCInfo->m_motionType = hkpMotion::MotionType::MOTION_KEYFRAMED;
		handCollCInfo->m_enableDeactivation = false;
		handCollCInfo->m_solverDeactivation = hkpRigidBodyCinfo::SolverDeactivation::SOLVER_DEACTIVATION_OFF;

		hkpRigidBody_ctor(handCollBody, handCollCInfo);
		hkpWorld_AddEntity(world->world, handCollBody, 1);
		_MESSAGE("Adding collision for hand");
	}

	// Put our hand collision where we want it
	NiPoint3 desiredPos = (handNode->m_worldTransform * (NiPoint3(0, -0.005, 0.08) / havokWorldScale)) * havokWorldScale;
	hkRotation desiredRot;
	NiMatrixToHkMatrix(handNode->m_worldTransform.rot, desiredRot);
	hkQuaternion desiredQuat;
	desiredQuat.setFromRotationSimd(desiredRot);
	hkpKeyFrameUtility_applyHardKeyFrame(NiPointToHkVector(desiredPos), desiredQuat, 1.0f / *g_deltaTime, handCollBody);


	NiPoint3 handPosRoomspace = wandNode->m_localTransform.pos;
	NiPoint3 handVelocityRoomspace = (handPosRoomspace - prevHandPosRoomspace) / *g_deltaTime;

	// Update velocities array to this frame
	for (int i = numPrevVel - 1; i >= 1; i--) {
		handVelocities[i] = handVelocities[i - 1];
	}
	handVelocities[0] = handVelocityRoomspace;

	NiPoint3 avgVelocityRoomspace = std::accumulate(handVelocities, &handVelocities[numPrevVel - 1], NiPoint3()) / numPrevVel;

	// Get whatever transform takes the wand position from room space to skyrim worldspace
	NiMatrix33 localToWorldTransform = wandNode->m_worldTransform.rot * wandNode->m_localTransform.rot.Transpose();
	NiMatrix33 worldToLocalTransform = localToWorldTransform.Transpose();

	NiPoint3 velocityWorldspace = localToWorldTransform * avgVelocityRoomspace;
	float handSpeedInSpellDirection = DotProduct(velocityWorldspace, castDirection);
	float handSpeedInObjDirection = 0;

	NiPoint3 handDirectionRoomspace = worldToLocalTransform * castDirection;
	NiPoint3 deltaHandDirectionRoomspace = handDirectionRoomspace - prevHandDirectionRoomspace;
	NiPoint3 handDirectionVelocityRoomspace = deltaHandDirectionRoomspace / *g_deltaTime;

	// Update velocities array to this frame
	for (int i = numPrevVel - 1; i >= 1; i--) {
		handDirectionVelocities[i] = handDirectionVelocities[i - 1];
	}
	handDirectionVelocities[0] = handDirectionVelocityRoomspace;

	NiPoint3 avgDirectionVelocityRoomspace = std::accumulate(handDirectionVelocities, &handDirectionVelocities[numPrevVel - 1], NiPoint3()) / numPrevVel;

	//if (std::string(handNodeName.data) == "NPC R Hand [RHnd]") {
	//	PrintToFile(std::to_string(VectorLength(avgDirectionVelocityRoomspace)), "hand_dir_speed_r.txt");
	//}

	NiAVObject *upperArmNode = player->GetNiRootNode(0)->GetObjectByName(&upperArmNodeName.data);
	if (!upperArmNode)
		return;

	NiPoint3 upperArmPos = upperArmNode->m_worldTransform.pos;

	bhkSimpleShapePhantom *sphere = *SPHERE_SHAPE_ADDR;
	if (!sphere)
		return;

	auto sphereShape = (hkpConvexShape *)sphere->phantom->m_collidable.m_shape;
	// Save sphere properties so we can change them and restore them later
	UInt32 filterInfoBefore = sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo;
	float radiusBefore = sphereShape->getRadius();
	hkVector4 translationBefore = sphere->phantom->m_motionState.getTransform().getTranslation();

	if (state == IDLE || state == SELECTED_CLOSE || state == SELECTED_FAR) {
		bool isSelectedNear = false;

		// See if there's something near the hand to pick up
		NiPointer<TESObjectREFR> closestObj;
		hkpCollidable *closestColl = nullptr;
		hkContactPoint closestPoint;

		NiAVObject *palmNode = player->GetNiRootNode(1)->GetObjectByName(&palmNodeName.data);
		NiPoint3 hkPalmNodePos = palmNode->m_worldTransform.pos * havokWorldScale;

		sphereShape->m_radius = (Config::options.grabRadius);
		sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = 0x2c;// 0x0009002C;
		sphere->phantom->m_motionState.m_transform.m_translation = NiPointToHkVector(hkPalmNodePos);

		cdPointCollector.reset();
		hkpWorld_GetClosestPoints(world->world, &sphere->phantom->m_collidable, world->world->m_collisionInput, &cdPointCollector);

		sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = filterInfoBefore;
		sphereShape->m_radius = radiusBefore;
		sphere->phantom->m_motionState.m_transform.m_translation = translationBefore;

		if (!cdPointCollector.m_hits.empty()) {
			bool didHitObject = false;
			float closestDistance = (std::numeric_limits<float>::max)();
			for (auto pair : cdPointCollector.m_hits) {
				auto collidable = static_cast<hkpCollidable *>(pair.first);
				if (!allowGrab || (other.HasExclusiveObject() && collidable == other.selectedObject.collidable)) {
					continue;
				}
				if (!hkpGetRigidBody(collidable)) {
					continue; // No rigidbody -> no movement :/
				}
				NiPointer<TESObjectREFR> ref = FindCollidableRef(collidable);
				if (ref) {
					if (IsAllowedCollidable(collidable) || ref->baseForm && ref->baseForm->formType == kFormType_Projectile) {
						if (ref->baseForm->formType == kFormType_Projectile) {
							auto impactData = *(void **)((UInt64)ref.m_pObject + 0x98);
							if (!impactData) {
								// Only grab projectiles that are not mid flight
								continue;
							}
						}
						hkContactPoint point = pair.second;
						if (point.getDistance() < closestDistance) {
							closestObj = ref;
							closestColl = collidable;
							closestDistance = point.getDistance();
							closestPoint = point;
							isSelectedNear = true;
						}
					}
				}
			}
		}

		if (!closestObj) {
			// Nothing close by the hand. Check for stuff pointing from from the palm

			// Convert hand position from skyrim coords to havok coords
			NiPoint3 hkHmdPos = hmdPos * havokWorldScale;
			NiPoint3 hkHandPos = handPos * havokWorldScale;

			NiPoint3 hkTargetPos = hkHandPos + castDirection * Config::options.castDistance;

			NiPoint3 hitPosition = { hkTargetPos.x, hkTargetPos.y, hkTargetPos.z };

			// First, raycast in the pointing direction
			rayHitCollector.reset();
			rayCastInput.m_filterInfo = ((UInt32)playerCollisionGroup << 16) | 0x28;
			rayCastInput.m_from = NiPointToHkVector(hkHandPos);
			rayCastInput.m_to = NiPointToHkVector(hkTargetPos);
			hkpWorld_CastRay(world->world, &rayCastInput, &rayHitCollector);
			if (rayHitCollector.m_doesHitExist) {
				// If raycast hit, we want to linearcast only up to the ray hit location
				hitPosition = hkHandPos + (hkTargetPos - hkHandPos) * rayHitCollector.m_closestHitInfo.m_hitFraction;
			}

			// Now, linearcast up to the point the raycast hit, or up to the limit if it's empty space
			// 'CustomPick2' layer, to pick up projectiles, because ONLY THIS GODDAMN LAYER can collide with projectiles. This filterinfo will collide with everything.
			sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = 0x2C;
			sphereShape->m_radius = Config::options.castRadius;
			sphere->phantom->m_motionState.m_transform.m_translation = NiPointToHkVector(hkHandPos);

			linearCastInput.m_to = NiPointToHkVector(hitPosition);
			cdPointCollector.reset();
			hkpWorld_LinearCast(world->world, &sphere->phantom->m_collidable, &linearCastInput, &cdPointCollector, nullptr);

			sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = filterInfoBefore;
			sphereShape->m_radius = radiusBefore;
			sphere->phantom->m_motionState.m_transform.m_translation = translationBefore;

			// Process result of cast
			float closestDistance = (std::numeric_limits<float>::max)();
			for (auto pair : cdPointCollector.m_hits) {
				auto collidable = static_cast<hkpCollidable *>(pair.first);
				if (!allowGrab || (other.HasExclusiveObject() && collidable == other.selectedObject.collidable)) {
					continue;
				}
				if (!hkpGetRigidBody(collidable)) {
					continue; // No rigidbody -> no movement :/
				}
				NiPointer<TESObjectREFR> ref = FindCollidableRef(collidable);
				if (ref) {
					if (IsAllowedCollidable(collidable) || ref->baseForm && ref->baseForm->formType == kFormType_Projectile) {
						if (ref->baseForm->formType == kFormType_Projectile) {
							auto impactData = *(void **)((UInt64)ref.m_pObject + 0x98);
							if (!impactData) {
								// Only grab projectiles that are not mid flight
								continue;
							}
						}
						// Get distance from the hit on the collidable to the ray
						NiPoint3 hit = HkVectorToNiPoint(pair.second.getPosition());
						NiPoint3 handToHit = hit - hkHandPos;
						NiPoint3 handToHitAlongRay = castDirection * DotProduct(handToHit, castDirection); // project above vector onto ray
						float dist = VectorLength(handToHit - handToHitAlongRay); // distance from hit location to closest point on the ray
						if (dist < closestDistance && DotProduct(VectorNormalized(hit - hkHmdPos), hmdForward) >= Config::options.requiredCastDotProduct) {
							closestObj = ref;
							closestColl = collidable;
							closestDistance = dist;
						}
					}
				}
			}
		}

		// Check if we should select something new. If yes, stay in SELECTED but select the new object
		bool isSelectedThisFrame = false;
		UInt32 prevSelectedHandle = selectedObject.handle;
		if (closestObj) {
			NiPointer<TESObjectREFR> selectedObj;
			if (!LookupREFRByHandle(selectedObject.handle, selectedObj) || closestObj != selectedObj) {
				if (selectedObj) {
					// Deselect the old thing if something else was selected
					StopShader(selectedObject.handle, selectedObject.shaderNode);
					Deselect();
				}
				// select the new refr
				Select(closestObj);
				state = isSelectedNear ? SELECTED_CLOSE : SELECTED_FAR;
			}
			else if ((state == SELECTED_FAR && isSelectedNear) || (state == SELECTED_CLOSE && !isSelectedNear)) {
				// Same object is selected, but it has become near/far enough to switch states
				state = isSelectedNear ? SELECTED_CLOSE : SELECTED_FAR;
			}

			// Figure out which node we should be playing a shader on, and switch to that one
			NiAVObject *nodeOnWhichToPlayShader = nullptr;
			Actor *actor = DYNAMIC_CAST(closestObj, TESObjectREFR, Actor);
			bool breakStickiness = false;

			if (actor) {
				NiAVObject *hitNode = FindCollidableNode(closestColl);
				if (hitNode) {
					BipedModel *biped = actor->GetBipedSmall();
					if (biped) {
						Biped *bipedData = biped->bipedData;
						if (bipedData) {
							int hitIndex = -1;
							TESForm *hitForm = nullptr;
							for (int i = 0; i < equippedWeaponSlotBase; i++) {
								// For skinned armor, the nodes are not attached to the skeleton. Find nodes the armor is skinned to and see if one of them was hit
								NiAVObject *geomNode = bipedData->unk10[i].object;
								if (geomNode) {
									TESForm *armorForm = bipedData->unk10[i].armor;
									if (armorForm) {
										// Now check if we actually hit a node the armor is skinned to
										bool isArmorHit = IsNodeWithinArmor(geomNode, hitNode);
										if (isArmorHit) {
											if (hitIndex == -1 || CompareBipedIndices(hitIndex, i)) {
												hitIndex = i;
												hitForm = armorForm;
											}
										}
									}
								}
							}
							for (int i = equippedWeaponSlotBase; i < 42; i++) {
								// For equipped weapons, the nodes are attached to the skeleton. Find the nearest parent that has collision and see if it was hit
								NiAVObject *geomNode = bipedData->unk10[i].object;
								bool hasCollision = false;
								if (geomNode) {
									if (DoesNodeHaveNode(geomNode, hitNode)) {
										// We collided with the weapon, so it must not be attached to the character
										hitIndex = i;
										hitForm = bipedData->unk10[i].armor;
										break;
									}
									else {
										// Weapon is attached to the character - get the nearest parent node that does have collision
										NiAVObject *nodeWithCollision = geomNode;
										while (nodeWithCollision) {
											if (nodeWithCollision->unk040) {
												hasCollision = true;
												break;
											}
											nodeWithCollision = nodeWithCollision->m_parent;
										}
										if (hasCollision && nodeWithCollision == hitNode) {
											hitIndex = i;
											hitForm = bipedData->unk10[i].armor;
											break;
										}
									}
								}
							}
							if (hitForm) {
								// Make sure the armor we hit is actually equipped. When nothing is equipped, there can still be 'naked' armor in the biped data that's not really equipped armor.
								ExtraContainerChanges* containerChanges = static_cast<ExtraContainerChanges*>(actor->extraData.GetByType(kExtraData_ContainerChanges));
								if (containerChanges) {
									MatchByForm matcher(hitForm);
									EquipData equipData = containerChanges->FindEquipped(matcher);
									TESForm *equippedForm = equipData.pForm;
									if (equippedForm) {
										auto hitBipedData = &bipedData->unk10[hitIndex];
										nodeOnWhichToPlayShader = hitBipedData->object;
										selectedObject.hitForm = hitForm;
										selectedObject.hitNode = hitNode;
									}
									else {
										// The form we hit is not equipped - do the same as if no form was selected
										breakStickiness = true;
									}
								}
							}
						}
					}
				}
			}
			if (nodeOnWhichToPlayShader) {
				if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
					if (nodeOnWhichToPlayShader != selectedObject.shaderNode) {
						// New node, same (or new) object
						TESEffectShader *shader;
						if (CALL_MEMBER_FN(selectedObj, IsOffLimits)()) {
							shader = itemSelectedShaderOffLimits;
						}
						else {
							shader = itemSelectedShader;
						}

						if (selectedObject.handle == prevSelectedHandle) {
							// Stop the shader on the current reference, we've changed nodes
							StopShader(selectedObject.handle, selectedObject.shaderNode);
						}
						else {
							// Stop the shader on the previous reference
							NiPointer<TESObjectREFR> prevSelectedObj;
							if (LookupREFRByHandle(prevSelectedHandle, prevSelectedObj)) {
								StopShader(prevSelectedHandle, selectedObject.shaderNode);
							}
						}
						PlayShader(selectedObject.handle, nodeOnWhichToPlayShader, shader);
						selectedObject.shaderNode = nodeOnWhichToPlayShader;
					}
				}
			}
			else if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				// No node selected but refr is still selected

				if (selectedObject.handle != prevSelectedHandle) {
					// New refr is selected. Stopping the shader for the previous ref is done earlier.
					TESEffectShader *shader;
					if (CALL_MEMBER_FN(selectedObj, IsOffLimits)()) {
						shader = itemSelectedShaderOffLimits;
					}
					else {
						shader = itemSelectedShader;
					}

					PlayShader(selectedObject.handle, nullptr, shader);

					selectedObject.shaderNode = nullptr;
					selectedObject.hitNode = nullptr;
					selectedObject.hitForm = nullptr;
				}
				else if (selectedObject.shaderNode) {
					// Node was selected before (selectedObject.shaderNode), but not now (nodeOnWhichToPlayShader). Stop the shader on that node.
					if (breakStickiness) {
						StopShader(selectedObject.handle, selectedObject.shaderNode);

						TESEffectShader *shader;
						if (CALL_MEMBER_FN(selectedObj, IsOffLimits)()) {
							shader = itemSelectedShaderOffLimits;
						}
						else {
							shader = itemSelectedShader;
						}

						PlayShader(selectedObject.handle, nullptr, shader);

						selectedObject.shaderNode = nullptr;
						selectedObject.hitNode = nullptr;
						selectedObject.hitForm = nullptr;
					}
				}
			}

			// Set selected collidable no matter what, as we can have objects with more than one collidable
			selectedObject.collidable = closestColl;
			hkpRigidBody *rb = hkpGetRigidBody(closestColl);
			selectedObject.rigidBody = rb;

			isSelectedThisFrame = true;
			lastSelectedTime = g_currentFrameTime;
		}

		if (state == SELECTED_CLOSE || state == SELECTED_FAR) {
			NiPointer<TESObjectREFR> selectedObj;
			if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				if (LookupREFRByHandle(selectedObject.handle, selectedObj) && !isSelectedThisFrame && g_currentFrameTime - lastSelectedTime > Config::options.selectedLeewayTime) {
					// If time has run out and nothing is selected, deselect whatever is selected
					StopShader(selectedObject.handle, selectedObject.shaderNode);
					Deselect();
				}

				// Check if we should grab the object. If yes, grab and go to GRABBED
				if (triggerPressed && g_currentFrameTime - triggerPressedTime <= Config::options.triggerPressedLeewayTime) {
					if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
						if (state == SELECTED_FAR) {
							hkVector4 translation = selectedObject.rigidBody->m_motion.m_motionState.m_transform.m_translation;
							NiPoint3 hkHandPos = handPos * havokWorldScale;
							NiPoint3 hkObjPos = HkVectorToNiPoint(translation);
							NiPoint3 relObjPos = hkObjPos - hkHandPos;

							NiPoint3 forward = castDirection;
							NiPoint3 worldUp = { 0, 0, 1 };
							NiPoint3 right = CrossProduct(forward, worldUp);
							NiPoint3 up = CrossProduct(right, forward);
							initialObjPosRaySpace = { DotProduct(relObjPos, forward), DotProduct(relObjPos, right) , DotProduct(relObjPos, up) };
							selectionLockedTime = g_currentFrameTime;
							initialGrabbedObjRelativePosition = relObjPos;
							initialGrabbedObjWorldPosition = hkObjPos;
							initialHandShoulderDistance = VectorLength(handPos - upperArmPos);
							state = SELECTION_LOCKED;
						}
						else if (state == SELECTED_CLOSE) {
							NiAVObject *n = FindCollidableNode(selectedObject.collidable);
							if (n) {
								StopShader(selectedObject.handle, selectedObject.shaderNode);

								NiPoint3 ptPos = HkVectorToNiPoint(closestPoint.getPosition());
								//NiPoint3 normal = HkVectorToNiPoint(closestPoint.m_separatingNormal); // vec from sphere center to point

								// First, raycast in the pointing direction
								bool didRayHitObject = false;
								NiPoint3 rayNormal;
								allRayHitCollector.reset();
								//NiPoint3 hkTargetPos = hkPalmNodePos + castDirection * (Config::options.grabRadius * 2);
								NiPoint3 hkTargetPos = hkPalmNodePos + (ptPos - hkPalmNodePos) * 1.5;
								rayCastInput.m_filterInfo = ((UInt32)playerCollisionGroup << 16) | 0x28;
								rayCastInput.m_from = NiPointToHkVector(hkPalmNodePos);
								rayCastInput.m_to = NiPointToHkVector(hkTargetPos);

								//hkpShapeRayCastInput input;
								//hkpShapeRayCastOutput output;
								//input.m_from = NiPointToHkVector(hkPalmNodePos);
								//input.m_to = NiPointToHkVector(hkTargetPos);
								//input.m_rayShapeCollectionFilter = world->world->m_collisionFilter;
								//UInt64 *vtbl = *((UInt64 **)selectedObject.collidable->m_shape);
								//((_hkpShape_castRayImpl)(vtbl[0x08]))(selectedObject.collidable->m_shape, input, output);

								//if (output.m_hitFraction < 1.0f) {
								//	didRayHitObject = true;
								//	rayNormal = HkVectorToNiPoint(output.m_normal);
								//}

								hkpWorld_CastRay(world->world, &rayCastInput, &allRayHitCollector);
								if (!allRayHitCollector.m_hits.empty()) {
									for (auto pair : allRayHitCollector.m_hits) {
										auto body = pair.first;
										if (body == selectedObject.collidable) {
											didRayHitObject = true;
											rayNormal = HkVectorToNiPoint(pair.second.m_normal);
											break;
										}
									}
								}

								if (didRayHitObject) {
									_MESSAGE("Ray hit");
									_MESSAGE("%.2f", DotProduct(rayNormal, castDirection));
								}

								NiPoint3 centerOfMass = HkVectorToNiPoint(selectedObject.rigidBody->m_motion.m_motionState.m_transform.m_translation);
								NiPoint3 ptToCenter = centerOfMass - ptPos; // in hk coords
								NiPoint3 desiredPos = palmNode->m_worldTransform.pos + (ptToCenter / havokWorldScale); // in skyrim coords
								NiTransform desiredTransform = n->m_worldTransform;
								desiredTransform.pos = desiredPos;
								NiTransform inverseHand;
								handNode->m_worldTransform.Invert(inverseHand);
								initialObjTransformHandSpace = inverseHand * desiredTransform;
								state = HELD;
							}
						}
						// Set to false only here, so that you can hold the trigger until the cast hits something valid
						triggerPressed = false;
						didTriggerPressGrabObject = true; // This variable is not set to false when we push/pull the object
					}
					else {
						// Selected object no longer exists
						state = IDLE;
					}
				}
			}
			else {
				state = IDLE;
			}
		}

		if (triggerReleased) {
			triggerReleased = false;
			didTriggerPressGrabObject = false;
		}
	}

	NiPointer<TESObjectREFR> selectedObj;
	if (state == SELECTION_LOCKED || state == GRABBED || state == PULLED || state == HELD) {
		// Check if we should drop the object. If yes, go to IDLE
		if (triggerReleased) {
			triggerReleased = false;
			didTriggerPressGrabObject = false;

			NiPointer<TESObjectREFR> selectedObj;
			if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				if (selectedObject.hasSavedAngularDamping) {
					selectedObject.rigidBody->m_motion.m_motionState.m_angularDamping = selectedObject.savedAngularDamping;
					selectedObject.hasSavedAngularDamping = false;
				}

				StopShader(selectedObject.handle, selectedObject.shaderNode);
				if (selectedObject.rigidBody) {
					if (selectedObject.hasSavedCollisionFilterInfo) {
						//selectedObject.hasSavedCollisionFilterInfo = false;
						// TODO: We no longer set the collisionfilterinfo back so that it doesn't collide with the player anymore. Should it be this way?
						//selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo = selectedObject.savedCollisionFilterInfo;
					}
					if (selectedObject.hasSavedMotionType) {
						selectedObject.hasSavedMotionType = false;
						selectedObject.hasSavedCollisionFilterInfo = false;
						// Restore only the original layer first, so it collides with everything except the player
						selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~0x7f;
						selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (selectedObject.savedCollisionFilterInfo & 0x7f);
						selectedObject.collidable->m_broadPhaseHandle.m_objectQualityType = HK_COLLIDABLE_QUALITY_CRITICAL; // Will make object collide with other things as motion type is changed
						hkpRigidBody_setMotionType(selectedObject.rigidBody, selectedObject.savedMotionType, 1, 0);

						selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo = selectedObject.savedCollisionFilterInfo;
						selectedObject.collidable->m_broadPhaseHandle.m_objectQualityType = selectedObject.savedQuality;
						hkpRigidBody_setMotionType(selectedObject.rigidBody, selectedObject.savedMotionType, 1, 0);
					}
					else {
						if (selectedObject.hasSavedCollisionFilterInfo) {
							selectedObject.hasSavedCollisionFilterInfo = false;
							selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo = selectedObject.savedCollisionFilterInfo;
							hkpWorld_UpdateCollisionFilterOnEntity(world->world, selectedObject.rigidBody, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);
						}
					}

					if (state == HELD) {
						//selectedObject.rigidBody->motion.m_linearVelocity = NiPointToHkVector((localToWorldTransform * handVelocityRoomspace) * *HAVOK_WORLD_SCALE_ADDR);
						// Boost the velocity a bit
						// TODO: Do a bit of lookback, we probably want the velocity from some time before we actually let go of the object
						selectedObject.rigidBody->m_motion.m_linearVelocity = NiPointToHkVector(velocityWorldspace * 1.5f * *HAVOK_WORLD_SCALE_ADDR);
					}
				}
				Deselect();
			}
		}

		NiPoint3 hkHandPos = handPos * havokWorldScale;

		if (state == SELECTION_LOCKED || state == GRABBED) {
			if (LookupREFRByHandle(selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {
				hkpMotion *motion = &selectedObject.rigidBody->m_motion;

				hkVector4 translation = motion->m_motionState.m_transform.m_translation;

				NiPoint3 hkObjPos = HkVectorToNiPoint(translation);

				NiPoint3 relObjPos = hkObjPos - hkHandPos;

				handSpeedInObjDirection = DotProduct(velocityWorldspace, VectorNormalized(relObjPos));
				//if (std::string(handNodeName.data) == "NPC R Hand [RHnd]") {
				//	PrintToFile(std::to_string(VectorLength(handDirectionVelocityRoomspace)), "hand_delta_dir_r.txt");
				//	PrintToFile(std::to_string(handSpeedInObjDirection), "hand_speed_r.txt");
				//}

				auto TransitionGrabbedPulled = [this, motion, relObjPos, hkObjPos, handPos, upperArmPos, selectedObj, playerCollisionGroup]()
				{
					grabbedTime = g_currentFrameTime;
					initialGrabbedObjRelativePosition = relObjPos;
					initialGrabbedObjWorldPosition = hkObjPos;
					initialHandShoulderDistance = VectorLength(handPos - upperArmPos);

					if (selectedObject.isImpactedProjectile) { // It's an embedded projectile, i.e. stuck in a wall etc.
						auto collObj = (bhkCollisionObject *)selectedObj->loadedState->node->unk040;
						if (collObj) {
							// Do not use grabbedObject.collidable here, as sometimes we end up grabbing the phantom shape of the projectile instead of the 3D one
							auto collidable = &collObj->body->hkBody->m_collidable;
							// Projectiles do not interact with collision usually. We need to change the filter to make them interact.
							collidable->m_broadPhaseHandle.m_collisionFilterInfo = (((UInt32)playerCollisionGroup) << 16) | 6; // player collision group, 'projectile' collision layer
							// Projectiles have 'Fixed' motion type by default, making them unmovable
							hkpRigidBody_setMotionType(collObj->body->hkBody, hkpMotion::MotionType::MOTION_DYNAMIC, 1, 0);
						}
					}
				};

				auto TransitionGrabbed = [this, TransitionGrabbedPulled]
				{
					state = GRABBED;
					//hkpRigidBody_setMotionType(selectedObject.rigidBody, 4, 1, 1);
					TransitionGrabbedPulled();
				};

				auto TransitionPulled = [this, TransitionGrabbedPulled, motion, selectedObj]()
				{
					state = PULLED;

					StopShader(selectedObject.handle, selectedObject.shaderNode);

					selectedObject.hasSavedAngularDamping = true;
					selectedObject.savedAngularDamping = motion->m_motionState.m_angularDamping;
					motion->m_motionState.m_angularDamping = hkHalf(3.0f);
					if (selectedObject.hitForm) {
						Actor *actor = DYNAMIC_CAST(selectedObj, TESObjectREFR, Actor);
						if (actor) {
							// drop the armor
							ExtraContainerChanges* containerChanges = static_cast<ExtraContainerChanges*>(actor->extraData.GetByType(kExtraData_ContainerChanges));
							if (containerChanges) {
								MatchByForm matcher(selectedObject.hitForm);
								EquipData equipData = containerChanges->FindEquipped(matcher);
								TESForm *itemForm = equipData.pForm;
								BaseExtraList *armorExtraData = equipData.pExtraData;
								if (itemForm) {
									TESBoundObject *item = DYNAMIC_CAST(itemForm, TESForm, TESBoundObject);
									if (item) {
										_MESSAGE("Dropping item");

										// pump armor form / extra data into actor->RemoveItem (vfunc 0x56)
										// Actor::RemoveItem is at 0x607F60

										UInt64 *vtbl = *((UInt64 **)actor);
										UInt32 droppedObjHandle = *g_invalidRefHandle;
										// TODO: For dropped weapons, make the drop pos / rot equal to where it was before
										NiPoint3 dropLoc = selectedObject.hitNode->m_worldTransform.pos + NiPoint3(0, 0, 30); // move it up a bit
										((_RemoveItem)(vtbl[0x56]))(actor, &droppedObjHandle, item, 1, 3, armorExtraData, nullptr, &dropLoc, nullptr);

										NiPointer<TESObjectREFR> droppedObj;
										if (LookupREFRByHandle(droppedObjHandle, droppedObj)) {
											selectedObject.handle = droppedObjHandle;
											selectedObject.collidable = nullptr;
											selectedObject.rigidBody = nullptr;

											// Remove ownership so it doesn't count as stealing
											BSExtraData *ownershipData = droppedObj->extraData.GetByType(kExtraData_Ownership);
											if (ownershipData) {
												ExtraOwnership *ownership = DYNAMIC_CAST(ownershipData, BSExtraData, ExtraOwnership);
												if (ownership) {
													ownership->owner = nullptr;
												}
											}
										}
									}
								}
							}
						}
					}
					TransitionGrabbedPulled();
				};

				// Determine the position where we want the object to be
				// Essentially it's a cylinder with a radius of the original distance when we grabbed it, and a height determined by some limit
				float maxHeight = selectedObject.isActor ? Config::options.maxBodyHeight : Config::options.maxItemHeight;
				NiPoint3 horiz;
				float h;
				if (castDirection.z <= 0.9999f) {
					float w = VectorLength(NiPoint3(initialGrabbedObjRelativePosition.x, initialGrabbedObjRelativePosition.y, 0)); // horizontal distance from hand to object
					float theta = asinf(castDirection.z); // vertical angle of hand relative to horizontal
					float tanTheta = tanf(theta);
					h = min(w * tanTheta, maxHeight); // desired height relative to horizontal from hand
					horiz = VectorNormalized(NiPoint3(castDirection.x, castDirection.y, 0)) * (h / tanTheta); // desired horizontal position relative to hand
				}
				else {
					// Degenerate case
					h = maxHeight;
					horiz = { 0, 0, 0 };
				}

				NiPoint3 desiredPos = NiPoint3(horiz.x, horiz.y, h);

				// Use distance from upper arm (shoulder-ish) to hand to control how far away from us we want the object to be
				float handShoulderDistance = VectorLength(handPos - upperArmPos);
				float handDistanceRatio = (handShoulderDistance - 10.0f) / (initialHandShoulderDistance - 10.0f);
				desiredPos *= handDistanceRatio;

				NiPoint3 deltaPos = desiredPos - relObjPos;

				if (state == SELECTION_LOCKED) {
					NiPoint3 linearVelocity = HkVectorToNiPoint(motion->m_linearVelocity);
					NiPoint3 angularVelocity = HkVectorToNiPoint(motion->m_angularVelocity);
					// TODO: Config all these thresholds
					if (VectorLength(linearVelocity) > 0.1f || VectorLength(angularVelocity) > 0.1f) {
						TransitionGrabbed();
					}
					else {
						bool grab = false, pull = false;
						if ((VectorLength(avgDirectionVelocityRoomspace) > Config::options.pullAngularSpeedThreshold && handSpeedInObjDirection < 0) || handSpeedInObjDirection < -Config::options.pushPullSpeedThreshold) {
							if (IsObjectPullable()) {
								pull = true;
							}
						}
						if (!pull) {
							// Hold object still while hand doesn't point too far away from original location. If far enough, transition to grabbed.
							NiPoint3 forward = castDirection;
							NiPoint3 worldUp = { 0, 0, 1 };
							NiPoint3 right = CrossProduct(forward, worldUp);
							NiPoint3 up = CrossProduct(right, forward);
							NiPoint3 initialObjPos = forward * initialObjPosRaySpace.x + right * initialObjPosRaySpace.y + up * initialObjPosRaySpace.z;
							if (DotProduct(VectorNormalized(initialObjPos), VectorNormalized(relObjPos)) < Config::options.grabbedDotProductThreshold || abs(DotProduct(deltaPos, castDirection)) > 1.5f) {
								grab = true;
							}
						}

						if (pull) {
							TransitionPulled();
						}
						else if (grab) {
							TransitionGrabbed();
						}
					}
				}

				if (state == GRABBED) {
					// Check if we should push or pull. If pull, go to PULLED. If push, push it and go to IDLE

					bool push = false, pull = false;
					if ((VectorLength(avgDirectionVelocityRoomspace) > Config::options.pullAngularSpeedThreshold && handSpeedInObjDirection < 0) || handSpeedInObjDirection < -Config::options.pushPullSpeedThreshold) {
						if (IsObjectPullable()) {
							pull = true;
						}
					}
					else if (abs(handSpeedInSpellDirection) < abs(prevHandSpeedInSpellDirection)) {
						// We've hit a local max in hand speeds

						// See if we passed the speed threshold. If we did, we want to push or pull.
						if (prevHandSpeedInSpellDirection > Config::options.pushPullSpeedThreshold) {
							push = true;
						}
					}

					if (push) {
						float inverseMass = min(motion->getMassInv(), Config::options.inverseMassLimit);

						if (selectedObject.isActor && selectedObj->loadedState && selectedObj->loadedState->node) {
							// For dead bodies, instead of the mass of the individual collidable we hit, use the summed mass over the entire body
							Actor *actor = DYNAMIC_CAST(selectedObj, TESObjectREFR, Actor);
							if (actor) {
								float mass = GetMass(0, false, actor);
								if (mass > 0) {
									inverseMass = 1.0f / mass;
								}
								else {
									auto actorBase = DYNAMIC_CAST(actor->baseForm, TESForm, TESActorBase);
									_WARNING("Could not get mass for actor: %s", actorBase->fullName.name.data);
								}
							}
						}

						float newMagnitude = (pow(inverseMass, Config::options.massExponent) * prevHandSpeedInSpellDirection * Config::options.pushVelocityMultiplier) / havokWorldScale;
						NiPoint3 newVelocity = VectorNormalized(velocityWorldspace) * newMagnitude;

						hkpEntity_activate(selectedObject.rigidBody);
						motion->m_linearVelocity = NiPointToHkVector(newVelocity);

						StopShader(selectedObject.handle, selectedObject.shaderNode);
						Deselect();
					}
					else if (pull) {
						TransitionPulled();
					}
					else {
						float inverseMass = min(motion->getMassInv(), Config::options.inverseMassLimit);

						if (selectedObject.isActor && selectedObj->loadedState && selectedObj->loadedState->node) {
							// For dead bodies, instead of the mass of the individual collidable we hit, use the summed mass over the entire body
							Actor *actor = DYNAMIC_CAST(selectedObj, TESObjectREFR, Actor);
							if (actor) {
								float mass = GetMass(0, false, actor);
								if (mass > 0) {
									inverseMass = 1.0f / mass;
								}
								else {
									auto actorBase = DYNAMIC_CAST(actor->baseForm, TESForm, TESActorBase);
									_WARNING("Could not get mass for actor: %s", actorBase->fullName.name.data);
								}
							}
						}

						float rampUp = min((g_currentFrameTime - grabbedTime) / Config::options.grabbedRampUpTime, 1.0f); // 0 to 1 over some number of ms
						//float newMagnitude = VectorLength(deltaPos) * pow(inverseMass, Config::options.massExponent) * Config::options.hoverVelocityMultiplier * rampUp;
						float newMagnitude = VectorLength(deltaPos) * 0.1 * Config::options.hoverVelocityMultiplier * rampUp;
						newMagnitude /= havokWorldScale;

						NiPoint3 newVelocity = VectorNormalized(deltaPos) * newMagnitude;

						hkpEntity_activate(selectedObject.rigidBody);
						motion->m_linearVelocity = NiPointToHkVector(newVelocity);
						//NiPoint3 desiredWorld = (hkHandPos + desiredPos) / havokWorldScale;
						//TESObjectREFR_SetPosition(selectedObj, desiredWorld);
						//UInt64 *vtbl = *((UInt64 **)selectedObj.m_pObject);
						//((_Update3DPosition)(vtbl[0x3F]))(selectedObj, 0);
					}
				}
			}
			else {
				// Grabbed object doesn't exist - go back to IDLE
				state = IDLE;
			}
		}

		if (state == PULLED) {
			if (LookupREFRByHandle(selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {
				if (!selectedObject.rigidBody) {
					auto collObj = (bhkCollisionObject *)selectedObj->loadedState->node->unk040;
					if (collObj) {
						selectedObject.rigidBody = collObj->body->hkBody;
						selectedObject.collidable = &selectedObject.rigidBody->m_collidable;
					}
				}

				if (!selectedObject.hasSavedCollisionFilterInfo) {
					selectedObject.hasSavedCollisionFilterInfo = true;
					selectedObject.savedCollisionFilterInfo = selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo;
					selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo &= 0x0000FFFF; // clear out collision group
					selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo |= ((UInt32)playerCollisionGroup) << 16;
					selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~0x7F; // clear out layer
					selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo |= 56; // our custom layer
					// set bit 15. This way it won't collide with the player, but _will_ collide with other objects that also have bit 15 set (i.e. other things we pick up).
					selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (1 << 15); // Why bit 15? It's just the way the collision works.
				}

				hkpMotion *motion = &selectedObject.rigidBody->m_motion;
				NiPoint3 hkObjPos = HkVectorToNiPoint(motion->m_motionState.m_transform.m_translation);

				float distanceFactor = VectorLength(hkHandPos - hkObjPos) / 5.0f;
				float duration = 0.375f + 0.375f * distanceFactor;

				/*
				NiPoint3 horizontalDelta =
				NiPoint3 velocity = horizontalDelta / duration;
				float verticalDelta = hkHandPos.z - initialGrabbedObjWorldPosition.z;
				velocity.z = 0.5f * 9.81f * duration + verticalDelta / duration;

				hkpEntity_activate(selectedObject.rigidBody);
				motion->m_linearVelocity = NiPointToHkVector(velocity);
				*/

				// Apply a predicted velocity to reach the destination, for a few frames after pulling starts.
				// Why for a few frames? Because then if it's next to something, it has a few frames to push it out of the way instead of just flopping right away
				if (pullFrameCounter < numPullFrames)
				{
					NiPoint3 horizontalDelta = hkHandPos - hkObjPos;
					horizontalDelta.z = 0;
					NiPoint3 velocity = horizontalDelta / duration;
					float verticalDelta = hkHandPos.z - hkObjPos.z;
					velocity.z = 0.5f * 9.81f * duration + verticalDelta / duration;

					hkpEntity_activate(selectedObject.rigidBody);
					motion->m_linearVelocity = NiPointToHkVector(velocity);
					pullFrameCounter++;
				}
				else {
					triggerReleased = true; // TODO: pretty hacky
					pullFrameCounter = 0;
				}
			}
		}

		if (state == HELD) {
			if (LookupREFRByHandle(selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {
				hkpMotion *motion = &selectedObject.rigidBody->m_motion;
				NiAVObject *n = FindCollidableNode(selectedObject.collidable);
				if (n) {
					if (!selectedObject.hasSavedMotionType) {
						selectedObject.hasSavedMotionType = true;
						selectedObject.savedMotionType = motion->m_type;
						selectedObject.savedQuality = selectedObject.collidable->m_broadPhaseHandle.m_objectQualityType;

						hkpRigidBody_setMotionType(selectedObject.rigidBody, hkpMotion::MotionType::MOTION_KEYFRAMED, 1, 1);

					}

					if (!selectedObject.hasSavedCollisionFilterInfo) {
						selectedObject.hasSavedCollisionFilterInfo = true;
						selectedObject.savedCollisionFilterInfo = selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo;
						selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo &= 0x0000FFFF;
						selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo |= ((UInt32)playerCollisionGroup) << 16;
						selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~0x7F; // clear out layer
						selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo |= 56; // our custom layer
						// set bit 15. This way it won't collide with the player, but _will_ collide with other objects that also have bit 15 set (i.e. other things we pick up).
						selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (1 << 15); // Why bit 15? It's just the way the collision works.
					}

					/*
					hkpEntity_activate(selectedObject.rigidBody);
					NiTransform newTransform = handNode->m_worldTransform * initialObjTransformHandSpace;
					NiPoint3 desiredPos = newTransform.pos * havokWorldScale;
					hkRotation desiredRot;
					NiMatrixToHkMatrix(newTransform.rot, desiredRot);
					hkQuaternion desiredQuat;
					desiredQuat.setFromRotationSimd(desiredRot);
					hkpKeyFrameUtility_applyHardKeyFrame(NiPointToHkVector(desiredPos), desiredQuat, 1.0f / *g_deltaTime, selectedObject.rigidBody);
					*/

					NiTransform inverseParent;
					n->m_parent->m_worldTransform.Invert(inverseParent);
					NiTransform newTransform = handNode->m_worldTransform * initialObjTransformHandSpace;
					n->m_localTransform = inverseParent * newTransform;
					NiAVObject::ControllerUpdateContext ctx;
					ctx.flags = 0x2000; // makes havok sim more stable?
					ctx.delta = 0;
					NiAVObject_UpdateObjectUpwards(n, &ctx);
				}
			}
			else {
				state = IDLE;
			}
		}
	}

	prevState = state;
	prevHandPosRoomspace = handPosRoomspace;
	prevHandSpeedInSpellDirection = handSpeedInSpellDirection;
	prevHandSpeedInObjDirection = handSpeedInObjDirection;
	prevHandDirectionRoomspace = handDirectionRoomspace;
}


void Grabber::ControllerStateUpdate(uint32_t unControllerDeviceIndex, vr_src::VRControllerState001_t *pControllerState)
{
	bool triggerDownBefore = triggerDown;

	// Check if the trigger is pressed
	if (pControllerState->ulButtonPressed & vr_src::ButtonMaskFromId(vr_src::EVRButtonId::k_EButton_SteamVR_Trigger)) {
		triggerDown = true;

		if (!triggerDownBefore) {
			triggerPressedTime = GetTime();
			triggerPressed = true;
			triggerReleased = false;
		}

		if (didTriggerPressGrabObject) {
			// If something is grabbed, disable the trigger
			pControllerState->ulButtonPressed &= ~vr_src::ButtonMaskFromId(vr_src::EVRButtonId::k_EButton_SteamVR_Trigger);
		}
		else {
			if (!triggerDownBefore) {
				// Trigger pressed but object is not grabbed (yet?). Do not unsheathe weapons until leeway time has passed.
				PlayerCharacter *pc = *g_thePlayer;
				if (pc && !pc->actorState.IsWeaponDrawn()) {
					unsheatheDesired = true;
				}
			}
		}
	}
	else {
		triggerDown = false;

		if (triggerDownBefore) {
			triggerReleased = true;
			triggerPressed = false;
		}
	}

	if (unsheatheDesired) {
		double currentTime = GetTime();
		if (currentTime - triggerPressedTime <= Config::options.triggerPressedLeewayTime) {
			// Suppress trigger press
			pControllerState->ulButtonPressed &= ~vr_src::ButtonMaskFromId(vr_src::EVRButtonId::k_EButton_SteamVR_Trigger);
		}
		else {
			unsheatheDesired = false;
			// Do the unsheathing, only if we didn't grab something
			if (!didTriggerPressGrabObject) {
				PlayerCharacter *pc = *g_thePlayer;
				if (pc && !pc->actorState.IsWeaponDrawn()) {
					pc->DrawSheatheWeapon(true);
				}
			}
		}
	}
}


bool Grabber::IsObjectPullable()
{
	NiPointer<TESObjectREFR> selectedObj;
	if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
		//return (selectedObj->baseForm && selectedObj->baseForm->formType != kFormType_MovableStatic && (!selectedObject.isActor || selectedObject.hitForm));
		return (!selectedObject.isActor || selectedObject.hitForm);
	}
	return false;
}


bool Grabber::HasExclusiveObject() const
{
	return state == GRABBED || state == PULLED || state == SELECTION_LOCKED || state == HELD;
}


bool Grabber::ShouldDisplayRollover()
{
	if (state != SELECTED_CLOSE && state != GRABBED && state != PULLED && state != SELECTION_LOCKED && state != HELD) return false;

	return IsObjectPullable();
}


void Grabber::SetupRollover(NiAVObject *rolloverNode, bool isLeftHanded)
{
	NiPointer<TESObjectREFR> selectedObj;
	if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
		// Give the hud with info about the object you're floating

		// First, change rotation/position/scale of the hud prompt
		rolloverNode->m_localTransform.pos = rolloverOffset;
		rolloverNode->m_localTransform.rot = rolloverRotation;
		rolloverNode->m_localTransform.scale = rolloverScale;

		// Now set all the places I could find that get set to the handle of the pointed at object usually
		UInt32 *selectedHandles = *SELECTED_HANDLES;
		if (selectedHandles) {
			if (isLeftHanded) {
				selectedHandles[1] = selectedObject.handle;
				selectedHandles[4] = selectedObject.handle;
				selectedHandles[7] = selectedObject.handle;
			}
			else {
				selectedHandles[2] = selectedObject.handle;
				selectedHandles[5] = selectedObject.handle;
				selectedHandles[8] = selectedObject.handle;
			}
		}
	}
}
