#include "RE/offsets.h"
#include "controller_tracking.h"
#include "hand.h"
#include "hand_broker.h"
#include "hand_observations.h"
#include "hand_parts_native.h"
#include "hand_physical_policy.h"
#include "hand_support.h"
#include "hand_transactions.h"
#include "own_contact.h"
#include "haptic_schedule.h"
#include "main.h"
#include "menu_checker.h"
#include "skse64/GameExtraData.h"
#include "skse64/GameRTTI.h"
#include "synergy_inventory_native.h"
#include "synergy_runtime.h"
#include <Physics/Collide/Query/CastUtil/hkpWorldRayCastInput.h>
#include <algorithm>
#include <cmath>

namespace HiggsHands
{
namespace
{
bool restoreConsume[2]{}, savedConsume[2]{};
struct NativeOperation
{
    uint64_t id = 0;
    HIGGS_HandOperationState state{};
    NiPointer<bhkRigidBody> support;
    NiPointer<TESObjectREFR> reference;
    const hkpShape *shape = nullptr;
    NiTransform originalGrip{};
    NiPointer<NiAVObject> supportNode;
    NiTransform supportWorld{};
    NiPointer<bhkWorld> supportWorldOwner;
    TESObjectWEAP *secondaryWeapon = nullptr;
    double secondaryGrabbedAt = 0;
    uint64_t secondaryEpisode = 0;
};
std::mutex nativeMutex;
NativeOperation targets[2]{}, grips[2]{};
uint64_t geometries[2]{};
std::vector<uint64_t> immediateConfirm;
uint64_t nextSupport = 1;
struct NativeItem
{
    uint64_t serial = 0, identity = 0;
    NiPointer<TESObjectREFR> reference;
};
std::vector<NativeItem> nativeItems;
uint64_t nativeItemSerial = 1, nativeItemEpoch = 0;
struct StaticSupportCollector : hkpRayHitCollector
{
    uint32_t wanted = 0;
    NiPointer<bhkRigidBody> body;
    NiPointer<TESObjectREFR> reference;
    float fraction = 1;
    NiPoint3 normal;
    explicit StaticSupportCollector(uint32_t id) : wanted(id)
    {
        m_earlyOutHitFraction = 1.f;
    }
    void addRayHit(const hkpCdBody &hitBody, const hkpShapeRayCastCollectorOutput &hit) override
    {
        if (!std::isfinite(hit.m_hitFraction) || hit.m_hitFraction < 0 || hit.m_hitFraction > 1)
            return;
        const hkpCdBody *root = &hitBody;
        while (root->m_parent)
            root = root->m_parent;
        if (hit.m_hitFraction >= fraction)
            return;
        fraction = hit.m_hitFraction;
        m_earlyOutHitFraction = fraction;
        body = nullptr;
        reference = nullptr;
        auto collidable = static_cast<const hkpCollidable *>(root);
        auto rigid = hkpGetRigidBody(collidable);
        // Cast the root and copy identity while the world read lock protects it.
        if (!rigid || !rigid->getUserData() || rigid->m_motion.m_type != hkpMotion::MotionType::MOTION_FIXED)
            return;
        auto ref = GetRefFromCollidable(collidable);
        if (!ref || ref->formID != wanted)
            return;
        body = static_cast<bhkRigidBody *>(reinterpret_cast<void *>(rigid->getUserData()));
        reference = ref;
        fraction = hit.m_hitFraction;
        normal = HkVectorToNiPoint(hit.m_normal);
        if (!std::isfinite(normal.x) || !std::isfinite(normal.y) || !std::isfinite(normal.z) ||
            VectorLength(normal) < 0.001f)
        {
            body = nullptr;
            reference = nullptr;
        }
        m_earlyOutHitFraction = fraction;
    }
};
NiTransform Transform(const HIGGS_HandOperation &request)
{
    NiTransform result{};
    result.pos = {request.position[0], request.position[1], request.position[2]};
    for (size_t r = 0; r < 3; ++r)
        for (size_t c = 0; c < 3; ++c)
            result.rot.data[r][c] = request.rotation[r * 3 + c];
    result.scale = request.scale;
    return result;
}
bool GripMatches(Hand *hand, const NativeOperation &grip)
{
    if (!hand || Service().HandGeneration(hand->isLeft ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT) !=
                     grip.state.operation.handGeneration)
        return false;
    if (grip.secondaryWeapon)
    {
        if (!hand->IsTwoHanding() || hand->synergySecondaryEpisode != grip.secondaryEpisode ||
            hand->twoHandedState.weapon != grip.secondaryWeapon || hand->grabbedTime != grip.secondaryGrabbedAt)
            return false;
    }
    else
    {
        NiPointer<TESObjectREFR> reference;
        if (!hand->HasHeldObject() || !LookupREFRByHandle(hand->selectedObject.handle, reference) || !reference ||
            reference->formID != grip.state.operation.itemRefId)
            return false;
    }
    const auto current = hand->GetGrabTransform(), wanted = Transform(grip.state.operation);
    if (!std::isfinite(current.pos.x) || !std::isfinite(current.pos.y) || !std::isfinite(current.pos.z) ||
        !std::isfinite(current.scale))
        return false;
    if (current.pos.x!=wanted.pos.x||current.pos.y!=wanted.pos.y||current.pos.z!=wanted.pos.z||current.scale!=wanted.scale)
        return false;
    for (size_t r = 0; r < 3; ++r)
        for (size_t c = 0; c < 3; ++c)
            if (!std::isfinite(current.rot.data[r][c]) || current.rot.data[r][c]!=wanted.rot.data[r][c])
                return false;
    return true;
}
bool DispatchOperation(uint64_t id, HIGGS_HandOperationState &state, void *)
{
    auto &p = state.operation;
    const size_t i = p.physicalHand == HIGGS_HAND_LEFT ? 0 : 1;
    Hand *hand = i == 0 ? g_leftHand : g_rightHand;
    auto player = *g_thePlayer;
    if (!hand || !hand->handBody || !player || !player->parentCell)
        return false;
    auto world = GetHavokWorldFromCell(player->parentCell);
    auto node = hand->GetFirstPersonHandNode();
    if (!world || !node)
        return false;
    const auto &transform = node->m_worldTransform;
    if (!std::isfinite(transform.pos.x) || !std::isfinite(transform.pos.y) || !std::isfinite(transform.pos.z) ||
        !std::isfinite(transform.scale) || transform.scale <= 0 || !std::isfinite(*g_havokWorldScale) ||
        *g_havokWorldScale <= 0)
        return false;
    for (size_t r = 0; r < 3; ++r)
        for (size_t c = 0; c < 3; ++c)
            if (!std::isfinite(transform.rot.data[r][c]))
                return false;
    if (p.kind == HIGGS_OP_HAPTIC)
    {
        HIGGS_HapticRequest request{};
        request.structSize = sizeof(request);
        request.physicalHand = p.physicalHand;
        request.epoch = state.epoch;
        request.durationMs = p.durationMs;
        request.startStrength = request.endStrength = p.hapticStrength;
        uint64_t scheduled = 0;
        if (Haptics().Request(state.owner, request, scheduled, Now(), id))
            return false;
        immediateConfirm.push_back(id);
        return true;
    }
    if (p.kind == HIGGS_OP_GRIP_TRANSFORM)
    {
        const bool secondary = hand->IsTwoHanding();
        if ((!hand->HasHeldObject() && !secondary) ||
            (secondary && (!p.grant || !hand->twoHandedState.weapon ||
                           hand->twoHandedState.weapon->formID != p.itemRefId || !hand->synergySecondaryEpisode ||
                           hand->synergySecondaryEpisode != Service().SecondaryEpisode(p.physicalHand))))
            return false;
        std::lock_guard lock(nativeMutex);
        if (grips[i].id)
        {
            if (Service().OperationActive(grips[i].id, Now()))
                return false;
            if (GripMatches(hand, grips[i]))
                hand->SetGrabTransform(grips[i].originalGrip);
            grips[i] = {};
        }
        grips[i].id = id;
        grips[i].state = state;
        grips[i].secondaryWeapon = secondary ? hand->twoHandedState.weapon : nullptr;
        grips[i].secondaryGrabbedAt = hand->grabbedTime;
        grips[i].secondaryEpisode = hand->synergySecondaryEpisode;
        grips[i].originalGrip = hand->GetGrabTransform();
        hand->SetGrabTransform(Transform(p));
        return true;
    }
    if (p.kind == HIGGS_OP_DRIVE_CAP)
        return hand->HasHeldConstrained() && hand->grabConstraint;
    if (p.kind == HIGGS_OP_HAND_GEOMETRY)
    {
        if (hand->HasHeldObject() || hand->IsTwoHanding())
            return false;
        const NiPoint3 extents{p.geometryHalfExtents[0], p.geometryHalfExtents[1], p.geometryHalfExtents[2]};
        BSWriteLocker lock(&world->worldLock);
        hand->RemoveHandCollision(world);
        hand->CreateHandCollision(world, &extents);
        if (!hand->handBody)
        {
            Service().BreakOperation(id, HIGGS_HAND_NOT_READY);
            hand->CreateHandCollision(world);
            return false;
        }
        geometries[i] = id;
        immediateConfirm.push_back(id);
        return true;
    }
    if (p.kind != HIGGS_OP_WORLD_SUPPORT && p.kind != HIGGS_OP_PHYSICAL_HAND_TARGET)
        return false;
    if (hand->HasHeldObject() || hand->IsTwoHanding())
        return false;
    const auto palm = hand->GetPalmPositionWS(node->m_worldTransform);
    const NiPoint3 desired{p.position[0], p.position[1], p.position[2]};
    if (VectorLength(desired - palm) > p.maxDistance)
        return false;
    NativeOperation target{};
    target.id = id;
    target.state = state;
    if (p.kind == HIGGS_OP_WORLD_SUPPORT)
    {
        const auto delta = desired - palm;
        if (VectorLength(delta) < 0.001f)
            return false;
        hkpWorldRayCastInput ray;
        ray.m_filterInfo = (static_cast<UInt32>(hand->playerCollisionGroup) << 16) | 0x28;
        ray.m_from = NiPointToHkVector(palm * *g_havokWorldScale);
        ray.m_to = NiPointToHkVector((desired + VectorNormalized(delta) * 0.5f) * *g_havokWorldScale);
        StaticSupportCollector hit(p.itemRefId);
        {
            BSReadLocker lock(&world->worldLock);
            hkpWorld_CastRay(world->world, &ray, &hit);
            if (hit.body && hit.reference && hit.body->GetHavokWorld_1() == world->world)
            {
                target.support = hit.body;
                target.reference = hit.reference;
                target.shape = hit.body->hkBody->getCollidable()->m_shape;
                target.supportNode = GetNodeFromCollidable(hit.body->hkBody->getCollidable());
                if (target.supportNode)
                    target.supportWorld = target.supportNode->m_worldTransform;
                target.supportWorldOwner = world;
            }
        }
        if (!target.support || !target.reference || !target.supportNode)
            return false;
        const auto anchor = palm + ((desired + VectorNormalized(delta) * 0.5f) - palm) * hit.fraction;
        if (VectorLength(anchor - palm) > p.maxDistance)
            return false;
        // Retain a fixed physical endpoint and target the existing keyframed hand;
        // no new dynamic constraint or locomotion policy is fabricated.
        auto pose = node->m_worldTransform;
        pose.pos += anchor - palm;
        for (size_t j = 0; j < 3; ++j)
            p.position[j] = j == 0 ? pose.pos.x : (j == 1 ? pose.pos.y : pose.pos.z);
        for (size_t r = 0; r < 3; ++r)
            for (size_t c = 0; c < 3; ++c)
                p.rotation[r * 3 + c] = pose.rot.data[r][c];
        p.scale = pose.scale;
        state.supportNormal[0] = hit.normal.x;
        state.supportNormal[1] = hit.normal.y;
        state.supportNormal[2] = hit.normal.z;
        state.supportGeneration = nextSupport++;
        state.supportPosition[0] = anchor.x;
        state.supportPosition[1] = anchor.y;
        state.supportPosition[2] = anchor.z;
    }
    target.state = state;
    {
        std::lock_guard lock(nativeMutex);
        targets[i] = target;
    }
    return true;
}
void ValidateOperations()
{
    Hand *hands[2]{g_leftHand, g_rightHand};
    auto player = *g_thePlayer;
    NiPointer<bhkWorld> world;
    if (player && player->parentCell)
        world = GetHavokWorldFromCell(player->parentCell);
    for (size_t i = 0; i < 2; ++i)
    {
        auto hand = hands[i];
        NativeOperation target, grip;
        {
            std::lock_guard lock(nativeMutex);
            target = targets[i];
            grip = grips[i];
        }
        if (target.id)
        {
            bool valid = Service().OperationActive(target.id, Now());
            if (valid && target.support)
            {
                if (!world || world != target.supportWorldOwner)
                    valid = false;
                else
                {
                    BSReadLocker worldLock(&world->worldLock);
                    valid = target.support->hkBody && target.support->GetHavokWorld_1() == world->world &&
                            target.support->hkBody->m_motion.m_type == hkpMotion::MotionType::MOTION_FIXED &&
                            target.support->hkBody->getCollidable()->m_shape == target.shape;
                    if (valid)
                    {
                        auto ref = LookupFormByID(target.state.operation.itemRefId);
                        valid = ref && ref == target.reference.operator->() && target.reference->GetNiNode();
                    }
                }
                if (valid)
                {
                    auto node = GetNodeFromCollidable(target.support->hkBody->getCollidable());
                    valid = node && node == target.supportNode &&
                            VectorLength(node->m_worldTransform.pos - target.supportWorld.pos) < 0.001f;
                    if (valid)
                        for (size_t r = 0; r < 3; ++r)
                            for (size_t c = 0; c < 3; ++c)
                                valid = valid && std::fabs(node->m_worldTransform.rot.data[r][c] -
                                                           target.supportWorld.rot.data[r][c]) < 0.0001f;
                }
            }
            if (valid && hand)
            {
                if (auto node = hand->GetFirstPersonHandNode())
                {
                    const auto p = node->m_worldTransform.pos;
                    const auto &op = target.state.operation;
                    valid =
                        VectorLength(p - NiPoint3(op.position[0], op.position[1], op.position[2])) <= op.maxDistance;
                }
                else
                    valid = false;
            }
            if (!valid)
            {
                Service().BreakOperation(target.id, HIGGS_HAND_STALE);
                std::lock_guard lock(nativeMutex);
                if (targets[i].id == target.id)
                    targets[i] = {};
            }
        }
        if (grip.id && Service().OperationActive(grip.id, Now()) && !GripMatches(hand, grip))
            Service().BreakOperation(grip.id, HIGGS_HAND_STALE);
        if (grip.id && !Service().OperationActive(grip.id, Now()))
        {
            if (GripMatches(hand, grip))
                hand->SetGrabTransform(grip.originalGrip);
            std::lock_guard lock(nativeMutex);
            if (grips[i].id == grip.id)
                grips[i] = {};
        }
        if (geometries[i] && !Service().OperationActive(geometries[i], Now()) && hand && world &&
            !hand->HasHeldObject() && !hand->IsTwoHanding())
        {
            BSWriteLocker worldLock(&world->worldLock);
            hand->RemoveHandCollision(world);
            hand->CreateHandCollision(world);
            geometries[i] = 0;
        }
    }
}
bool DispatchPhysical(const HIGGS_PhysicalRequest &request, void *)
{
    const size_t i = request.physicalHand == HIGGS_HAND_LEFT ? 0 : 1;
    Hand *hand = i == 0 ? g_leftHand : g_rightHand;
    if (!hand)
        return false;
    if (request.kind == HIGGS_PHYSICAL_GRAB)
    {
        auto form = LookupFormByID(request.itemRefId);
        auto ref = form ? DYNAMIC_CAST(form, TESForm, TESObjectREFR) : nullptr;
        if (!ref || !hand->CanGrabObject())
            return false;
        hand->externalGrabRequestedObject = ref;
        hand->externalGrabRequested = true;
        hand->synergyGrabPending = true;
        return true;
    }
    if (request.kind == HIGGS_PHYSICAL_SECONDARY_SUPPORT)
    {
        if (hand->state != Hand::State::SelectedTwoHand || !hand->CanTwoHand())
            return false;
        hand->grabRequested = true;
        hand->grabRequestedTime = g_currentFrameTime;
        hand->synergyGrabPending = true;
        return true;
    }
    if (request.kind == HIGGS_PHYSICAL_RELEASE && (hand->HasHeldObject() || hand->IsTwoHanding()))
    {
        PrepareSupportRelease(hand->isLeft);
        savedConsume[i] = hand->disableConsumeStash;
        restoreConsume[i] = true;
        hand->disableConsumeStash = true;
        hand->releaseRequested = true;
        return true;
    }
    return false;
}
} // namespace
void DrainNative()
{
    InvalidateNativeOwnContacts();
    Service().MarkMainThread();
    auto phase = Service().NativePhase();
    if (g_leftHand)
        Service().ObserveSecondaryEpisode(HIGGS_HAND_LEFT,
                                          g_leftHand->IsTwoHanding() ? g_leftHand->synergySecondaryEpisode : 0);
    if (g_rightHand)
        Service().ObserveSecondaryEpisode(HIGGS_HAND_RIGHT,
                                          g_rightHand->IsTwoHanding() ? g_rightHand->synergySecondaryEpisode : 0);
    MaintainNativeHandParts();
    MaintainNativePhysicalPolicies();
    ValidateOperations();
    Service().DrainHandUses();
    DrainNativeSupport();
    DrainNativeInventory();
    Transactions().Drain(Now());
    Service().DrainPhysical(DispatchPhysical, nullptr, Now());
    Service().DrainOperations(DispatchOperation, nullptr, Now());
    for (auto id : immediateConfirm)
        Service().ConfirmOperation(id, Now());
    immediateConfirm.clear();
}
void FinishNative()
{
    Hand *hands[2]{g_leftHand, g_rightHand};
    for (size_t i = 0; i < 2; ++i)
    {
        NiPointer<TESObjectREFR> held;
        const bool hasHeld = hands[i]->HasHeldObject();
        const bool known = !hasHeld || (LookupREFRByHandle(hands[i]->selectedObject.handle, held) && held);
        Service().EndHandMutation(i == 0 ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT, held ? held->formID : 0, known,
                                  hands[i]->IsTwoHanding(), Now());
        if (restoreConsume[i])
        {
            hands[i]->disableConsumeStash = savedConsume[i];
            restoreConsume[i] = false;
        }
    }
}
bool CopyNativeTarget(bool left, float position[3], float rotation[9], float &scale)
{
    if (CopySupportTarget(left, position, rotation, scale))
        return true;
    std::lock_guard lock(nativeMutex);
    const auto &target = targets[left ? 0 : 1];
    if (!target.id || !Service().OperationActive(target.id, Now()))
        return false;
    for (size_t j = 0; j < 3; ++j)
        position[j] = target.state.operation.position[j];
    for (size_t j = 0; j < 9; ++j)
        rotation[j] = target.state.operation.rotation[j];
    scale = target.state.operation.scale;
    return true;
}
bool HasNativeTarget(bool left)
{
    HIGGS_SupportState support{};
    if (Supports().Goal(left ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT, support, Now()))
        return true;
    std::lock_guard lock(nativeMutex);
    return targets[left ? 0 : 1].id && Service().OperationActive(targets[left ? 0 : 1].id, Now());
}
void ConfirmNativeTarget(bool left)
{
    ConfirmSupportTarget(left);
    std::lock_guard lock(nativeMutex);
    if (targets[left ? 0 : 1].id && Service().OperationActive(targets[left ? 0 : 1].id, Now()))
        Service().ConfirmOperation(targets[left ? 0 : 1].id, Now());
}
void ApplyDriveCap(bool left, float &linear, float &angular)
{
    HIGGS_HandOperationState state{};
    if (Service().CopyActiveOperation(left ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT, HIGGS_OP_DRIVE_CAP, state, Now()))
    {
        linear = (std::min)(linear, state.operation.linearForceCap);
        angular = (std::min)(angular, state.operation.angularForceCap);
        Service().ConfirmOperation(state.requestId, Now());
    }
}
bool CopyHandGeometry(bool left, float halfExtents[3])
{
    HIGGS_HandOperationState state{};
    if (!Service().CopyActiveOperation(left ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT, HIGGS_OP_HAND_GEOMETRY, state,
                                       Now()) ||
        !(state.flags & HIGGS_OP_APPLIED))
        return false;
    for (size_t i = 0; i < 3; ++i)
        halfExtents[i] = state.operation.geometryHalfExtents[i];
    return true;
}
void ConfirmNativeGrip(bool left)
{
    std::lock_guard lock(nativeMutex);
    const auto id = grips[left ? 0 : 1].id;
    if (id && Service().OperationActive(id, Now()) && GripMatches(left ? g_leftHand : g_rightHand, grips[left ? 0 : 1]))
        Service().ConfirmOperation(id, Now());
}
void PublishNative()
{
    auto publicationPhase = Service().NativePhase();
    if (g_leftHand)
        Service().ObserveSecondaryEpisode(HIGGS_HAND_LEFT,
                                          g_leftHand->IsTwoHanding() ? g_leftHand->synergySecondaryEpisode : 0);
    if (g_rightHand)
        Service().ObserveSecondaryEpisode(HIGGS_HAND_RIGHT,
                                          g_rightHand->IsTwoHanding() ? g_rightHand->synergySecondaryEpisode : 0);
    PlayerCharacter *player = *g_thePlayer;
    if (!player || !player->GetNiNode())
    {
        ObserveWorld(0);
        return;
    }
    const auto tracking = HiggsMotion::Tracking().CopyObservation();
    std::array<HIGGS_HandRecord, 2> records{};
    Hand *native[2]{g_leftHand, g_rightHand};
    for (size_t i = 0; i < 2; ++i)
    {
        auto &h = records[i];
        auto hand = native[i];
        if (!hand)
            continue;
        if (!hand->HasHeldObject())
            h.observationMask |= HIGGS_OBS_HELD_KNOWN;
        else
        {
            NiPointer<TESObjectREFR> held;
            if (LookupREFRByHandle(hand->selectedObject.handle, held) && held)
            {
                h.heldRefId = held->formID;
                h.observationMask |= HIGGS_OBS_HELD_KNOWN;
            }
        }
        h.observationMask |= HIGGS_OBS_EQUIPMENT_KNOWN;
        if (player->actorState.IsWeaponDrawn())
        {
            TESForm *equipped = player->GetEquippedObject(*g_leftHandedMode != hand->isLeft);
            TESObjectWEAP *main = GetEquippedWeapon(player, false);
            if (main && IsBow(main))
            {
                // Skyrim's dominant equipment slot holds a bow rendered in the other
                // physical hand. Its dominant hand instead uses the current arrow.
                if (hand->isLeft == *g_leftHandedMode)
                {
                    // CurrentAmmo is inventory selection. HIGGS has no qualified
                    // nocked-arrow visibility observation, so it cannot prove this
                    // physical hand empty or equipped merely from that pointer.
                    equipped = nullptr;
                    if (get_vfunc<Actor_GetCurrentAmmo>(player, 0x9F)(player))
                        h.observationMask &= ~HIGGS_OBS_EQUIPMENT_KNOWN;
                }
                else
                    equipped = main;
            }
            else if (main && IsTwoHanded(main) && hand->isLeft != *g_leftHandedMode)
                equipped = nullptr;
            if (auto weapon = equipped ? DYNAMIC_CAST(equipped, TESForm, TESObjectWEAP) : nullptr)
                if (weapon->gameData.type == TESObjectWEAP::GameData::kType_HandToHandMelee)
                    equipped = nullptr;
            if (equipped)
                h.equippedFormId = equipped->formID;
        }
        if (hand->IsTwoHanding())
            h.observationMask |= HIGGS_OBS_SECONDARY_SUPPORT;
        if (tracking.hands[i].available)
            h.observationMask |= HIGGS_OBS_TRACKING_VALID;
        if (auto node = hand->GetFirstPersonHandNode())
        {
            const auto p = hand->GetPalmPositionWS(node->m_worldTransform);
            h.palmPosition[0] = p.x;
            h.palmPosition[1] = p.y;
            h.palmPosition[2] = p.z;
            bool finite = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
            for (size_t row = 0; row < 3; ++row)
                for (size_t col = 0; col < 3; ++col)
                {
                    const auto value = node->m_worldTransform.rot.data[row][col];
                    h.palmRotation[row * 3 + col] = value;
                    finite = finite && std::isfinite(value);
                }
            if (finite)
                h.observationMask |= HIGGS_OBS_PALM_VALID;
        }
        if (HiggsSynergy::CopyHandState(hand->isLeft).handInteractionsDisabled)
            h.unavailableReasons |= HIGGS_HAND_REASON_RESTRICTION;
        if (MenuChecker::isGameStopped())
            h.unavailableReasons |= HIGGS_HAND_REASON_MENU;
        if (!(h.observationMask & HIGGS_OBS_TRACKING_VALID))
            h.unavailableReasons |= HIGGS_HAND_REASON_TRACKING;
        if (!hand->IsInGrabbableState() && !hand->HasHeldObject() && !hand->IsTwoHanding() && !hand->synergyGrabPending)
            h.unavailableReasons |= HIGGS_HAND_REASON_EXISTING_USE;
    }
    PublishNativeItems();
    Service().Publish(records, Now());
    PublishNativeOwnContacts();
    ObserveNativeSamples();
}
void PublishNativeItems()
{
    if (nativeItemEpoch != Service().Epoch())
    {
        nativeItems.clear();
        nativeItemEpoch = Service().Epoch();
    }
    uint64_t current[2]{};
    Hand *hands[2]{g_leftHand, g_rightHand};
    for (size_t i = 0; i < 2; ++i)
    {
        auto hand = hands[i];
        NiPointer<TESObjectREFR> ref;
        if (hand && hand->HasHeldObject() && LookupREFRByHandle(hand->selectedObject.handle, ref) && ref)
        {
            NativeItem *entry = nullptr;
            for (auto &item : nativeItems)
                if (item.reference == ref)
                {
                    entry = &item;
                    break;
                }
            if (!entry)
            {
                NativeItem item{};
                item.serial = nativeItemSerial++;
                item.reference = ref;
                nativeItems.push_back(item);
                entry = &nativeItems.back();
            }
            const auto count = BSExtraList_GetCount(&ref->extraData);
            entry->identity =
                Transactions().ObserveHeld(i == 0 ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT, entry->serial, ref->formID,
                                           ref->baseForm ? ref->baseForm->formID : 0, count, Now());
            current[i] = entry->identity;
        }
        else
            Transactions().ObserveHeld(i == 0 ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT, 0, 0, 0, 0, Now());
    }
    nativeItems.erase(std::remove_if(nativeItems.begin(), nativeItems.end(),
                                     [&](const NativeItem &item) {
                                         return item.identity != current[0] && item.identity != current[1] &&
                                                !Transactions().ItemInUse(item.identity) &&
                                                Transactions().RetireNativeItem(item.identity);
                                     }),
                      nativeItems.end());
}
bool CopyNativeItemReference(const HIGGS_ItemIdentity &identity, NiPointer<TESObjectREFR> &out)
{
    // Native inventory executor calls only from HIGGS main maintenance. The
    // retained object, rather than worldRefId lookup, protects release->inventory.
    auto phase = Service().NativePhase();
    if (identity.structSize < sizeof(identity) || identity.kind != HIGGS_ITEM_WORLD_REFERENCE ||
        !(identity.validFields & HIGGS_ITEM_NATIVE_OBSERVED) || identity.epoch != nativeItemEpoch ||
        identity.epoch != Service().Epoch() || !Transactions().IdentityCurrent(identity))
        return false;
    for (const auto &item : nativeItems)
        if (item.identity == identity.instance && item.reference)
        {
            auto ref = item.reference;
            if (ref->formID != identity.worldRefId || !ref->baseForm || ref->baseForm->formID != identity.baseFormId ||
                BSExtraList_GetCount(&ref->extraData) != identity.quantity)
                return false;
            out = ref;
            return true;
        }
    return false;
}
} // namespace HiggsHands
