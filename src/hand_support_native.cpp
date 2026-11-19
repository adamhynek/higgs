#include "RE/offsets.h"
#include "constraint.h"
#include "hand.h"
#include "hand_broker.h"
#include "hand_support.h"
#include "interop/planck_physical_interop.h"
#include "main.h"
#include "skse64/GameRTTI.h"
#include "skse64/PluginAPI.h"
#include <Physics/Dynamics/Constraint/Motor/Position/hkpPositionConstraintMotor.h>
#include <array>
#include <cmath>
#include <cstring>

namespace HiggsHands
{
namespace
{
struct Endpoint
{
    uint64_t id = 0, generation = 0, geometry = 0, revision = 0;
    PlanckToken lease = 0;
    bool invalid = false;
    NiPointer<bhkWorld> world;
    NiPointer<TESObjectREFR> reference;
    NiPointer<NiNode> model;
    NiPointer<bhkRigidBody> wrapper, source;
    RE::hkRefPtr<hkpRigidBody> body;
    RE::hkRefPtr<hkpShape> shape;
    NiPointer<bhkGroupConstraint> constraint;
};
std::mutex endpointMutex;
std::array<Endpoint, 64> endpoints{};
uint64_t nextEndpoint = 1;
PlanckSynergyAPI *planck = nullptr;
PlanckPhysicalInteropAPI *interop = nullptr;
PlanckToken planckClient = 0, planckSubscription = 0;
Endpoint *Find(uint64_t id)
{
    for (auto &endpoint : endpoints)
        if (endpoint.id == id)
            return &endpoint;
    return nullptr;
}
bool IsConstraint(uint32_t kind)
{
    return kind == HIGGS_CONSTRAINT_HELD_TO_WORLD || kind == HIGGS_CONSTRAINT_HELD_TO_PLANCK;
}
bool IsPlanck(uint32_t kind)
{
    return kind == HIGGS_SUPPORT_PLANCK_HAND || kind == HIGGS_CONSTRAINT_HELD_TO_PLANCK;
}
NiPoint3 Point(const float *p)
{
    return {p[0], p[1], p[2]};
}
void Store(float *out, const NiPoint3 &p)
{
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
}
bool Finite(const NiPoint3 &p)
{
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
PlanckBodyRef Body(const HIGGS_ProviderBodyRef &r)
{
    PlanckBodyRef body{};
    body.actor = {r.epoch, r.actorGeneration, r.actorHandle, 0};
    body.bodyGeneration = r.bodyGeneration;
    body.index = r.bodyIndex;
    return body;
}
void PLANCK_CALL Invalidated(const PlanckEvent *event, void *)
{
    if (!event)
        return;
    if (event->type != PLANCK_ENDPOINT_INVALIDATED && event->type != PLANCK_RESET)
        return;
    std::lock_guard lock(endpointMutex);
    for (auto &endpoint : endpoints)
        if (endpoint.id && endpoint.lease && (event->type == PLANCK_RESET || event->subject == endpoint.lease))
        {
            endpoint.invalid = true;
            // No engine calls, phase acquisition or registry calls under PLANCK's callback.
            Supports().Break(endpoint.id, HIGGS_SUPPORT_BREAK_ENDPOINT);
        }
}
bool SourceMatches(Endpoint &endpoint, const HIGGS_SupportState &state, Hand *hand)
{
    if (!hand || !hand->HasHeldObject() || !hand->selectedObject.rigidBody)
        return false;
    NiPointer<TESObjectREFR> reference;
    if (!LookupREFRByHandle(hand->selectedObject.handle, reference) || !reference ||
        reference->formID != state.request.sourceItemRefId)
        return false;
    auto source = hand->selectedObject.rigidBody;
    if (!source->hkBody || source->GetHavokWorld_1() != endpoint.world->world || source->hkBody == endpoint.body)
        return false;
    if (endpoint.source && endpoint.source != source)
        return false;
    endpoint.source = source;
    return true;
}
HIGGS_HandStatus ApplyLocked(Endpoint &endpoint, HIGGS_SupportState &state, hkpRigidBody *body, uint64_t geometry)
{
    // Caller owns graph/world (PLANCK) or current world write ownership (ordinary reference).
    auto hand = state.request.physicalHand == HIGGS_HAND_LEFT ? g_leftHand : g_rightHand;
    if (endpoint.invalid || !hand || !hand->handBody || !body || body->getWorld() != endpoint.world->world ||
        !body->getCollidable()->getShape())
        return HIGGS_HAND_STALE;
    auto node = hand->GetFirstPersonHandNode();
    if (!node || !std::isfinite(node->m_worldTransform.scale) || node->m_worldTransform.scale <= 0 ||
        !std::isfinite(*g_inverseHavokWorldScale) || *g_inverseHavokWorldScale <= 0)
        return HIGGS_HAND_NOT_READY;
    if (endpoint.body &&
        (endpoint.body != body || endpoint.shape != body->getCollidable()->getShape() || endpoint.geometry != geometry))
        return HIGGS_HAND_STALE;
    if (state.request.expectedGeometryRevision && state.request.expectedGeometryRevision != geometry)
        return HIGGS_HAND_STALE;
    if (!endpoint.body)
    {
        endpoint.body = body;
        endpoint.shape = const_cast<hkpShape *>(body->getCollidable()->getShape());
        endpoint.geometry = geometry;
        endpoint.generation = nextEndpoint++;
    }
    const auto bodyWorld = hkTransformToNiTransform(body->getTransform());
    const auto anchor = bodyWorld * (Point(state.request.targetLocalPoint) * *g_inverseHavokWorldScale);
    const auto palm = hand->GetPalmPositionWS(node->m_worldTransform);
    if (!Finite(anchor) || !Finite(palm) || VectorLength(anchor - palm) > state.request.maxDistanceWorld)
        return HIGGS_HAND_CONFLICT;
    Store(state.anchorPosition, anchor);
    state.endpointGeneration = endpoint.generation;
    state.geometryRevision = geometry;
    if (!IsConstraint(state.request.kind))
    {
        if (hand->HasHeldObject() || hand->IsTwoHanding())
            return HIGGS_HAND_CONFLICT;
        NiTransform local{};
        local.scale = 1;
        for (size_t r = 0; r < 3; ++r)
            for (size_t c = 0; c < 3; ++c)
                local.rot.data[r][c] = state.request.targetLocalRotation[r * 3 + c];
        const auto desiredRotation = bodyWorld.rot * local.rot;
        const auto position = anchor - (desiredRotation * hand->palmPosHandspace) * node->m_worldTransform.scale;
        Store(state.handPosition, position);
        for (size_t r = 0; r < 3; ++r)
            for (size_t c = 0; c < 3; ++c)
                state.handRotation[r * 3 + c] = desiredRotation.data[r][c];
        state.handScale = node->m_worldTransform.scale;
        state.flags |= HIGGS_SUPPORT_TARGET_VALID;
        return HIGGS_HAND_OK;
    }
    if (!SourceMatches(endpoint, state, hand))
        return HIGGS_HAND_STALE;
    const auto sourceWorld = hkTransformToNiTransform(endpoint.source->hkBody->getTransform());
    const auto sourceAnchor = sourceWorld * (Point(state.request.sourceLocalPoint) * *g_inverseHavokWorldScale);
    if (!Finite(sourceAnchor) || VectorLength(sourceAnchor - anchor) > state.request.maxDistanceWorld)
        return HIGGS_HAND_CONFLICT;
    NiTransform targetFrame{}, sourceFrame{};
    targetFrame.scale = sourceFrame.scale = 1;
    targetFrame.pos = Point(state.request.targetLocalPoint);
    sourceFrame.pos = Point(state.request.sourceLocalPoint);
    for (size_t r = 0; r < 3; ++r)
        for (size_t c = 0; c < 3; ++c)
            targetFrame.rot.data[r][c] = state.request.targetLocalRotation[r * 3 + c];
    if (!endpoint.constraint)
    {
        endpoint.constraint = CreateGrabConstraint(body, endpoint.source->hkBody, targetFrame, sourceFrame);
        if (!endpoint.constraint || !endpoint.constraint->constraint)
            return HIGGS_HAND_NOT_READY;
        bhkRigidBody_AddConstraintToArray(endpoint.source, endpoint.constraint);
        bhkWorld_AddConstraint(endpoint.world, endpoint.constraint->constraint);
    }
    if (!endpoint.constraint->constraint->getOwner())
        return HIGGS_HAND_STALE;
    auto data = const_cast<GrabConstraintData *>(
        static_cast<const GrabConstraintData *>(endpoint.constraint->constraint->getData()));
    if (!data)
        return HIGGS_HAND_NOT_READY;
    if (endpoint.revision != state.requestRevision)
    {
        data->setInBodySpace(NiTransformTohkTransform(targetFrame, false),
                             NiTransformTohkTransform(sourceFrame, false));
        data->setTargetRelativeOrientationOfBodies(NiTransformTohkTransform(sourceFrame, false).m_rotation);
        endpoint.revision = state.requestRevision;
    }
    for (unsigned i = 0; i < 3; ++i)
        static_cast<hkpPositionConstraintMotor *>(data->m_atoms.m_ragdollMotors.m_motors[i])
            ->setMaxForce(state.request.angularForceCap);
    static_cast<hkpPositionConstraintMotor *>(data->m_atoms.m_linearMotor0.m_motor)
        ->setMaxForce(state.request.linearForceCap);
    static_cast<hkpPositionConstraintMotor *>(data->m_atoms.m_linearMotor1.m_motor)
        ->setMaxForce(state.request.linearForceCap);
    static_cast<hkpPositionConstraintMotor *>(data->m_atoms.m_linearMotor2.m_motor)
        ->setMaxForce(state.request.linearForceCap);
    state.effectiveLinearCap = state.request.linearForceCap;
    state.effectiveAngularCap = state.request.angularForceCap;
    state.flags |= HIGGS_SUPPORT_CONSTRAINT_ATTACHED | HIGGS_SUPPORT_NATIVE_APPLIED | HIGGS_SUPPORT_DRIVE_LIMITED;
    return HIGGS_HAND_OK;
}
struct Visit
{
    uint64_t id;
    HIGGS_SupportState *state;
    HIGGS_HandStatus status = HIGGS_HAND_NOT_READY;
};
PlanckStatus PLANCK_CALL WithEndpoint(const PlanckNativeEndpoint *native, void *context)
{
    auto &visit = *static_cast<Visit *>(context);
    if (!native || native->size < sizeof(*native) || native->phase != PLANCK_PHASE_MAIN || !native->rigidBody ||
        !native->world)
        return PLANCK_STALE;
    std::lock_guard lock(endpointMutex);
    auto endpoint = Find(visit.id);
    if (!endpoint || endpoint->lease != native->endpoint.lease ||
        reinterpret_cast<hkpWorld *>(native->world) != endpoint->world->world.val())
        return PLANCK_STALE;
    visit.status = ApplyLocked(*endpoint, *visit.state, reinterpret_cast<hkpRigidBody *>(native->rigidBody),
                               native->geometryRevision);
    return visit.status == HIGGS_HAND_OK ? PLANCK_OK : PLANCK_STALE;
}
HIGGS_HandStatus Driver(HIGGS_SupportState &state, bool ending, void *)
{
    if (ending)
    {
        NiPointer<bhkWorld> world;
        PlanckToken lease = 0;
        {
            std::lock_guard lock(endpointMutex);
            auto endpoint = Find(state.requestId);
            if (!endpoint)
                return HIGGS_HAND_OK;
            world = endpoint->world;
            lease = endpoint->lease;
        }
        if (!world)
            return HIGGS_HAND_NOT_READY;
        {
            BSWriteLocker worldLock(&world->worldLock);
            std::lock_guard lock(endpointMutex);
            auto endpoint = Find(state.requestId);
            if (!endpoint)
                return HIGGS_HAND_OK;
            if (endpoint->constraint)
            {
                if (endpoint->constraint->constraint && endpoint->constraint->constraint->getOwner())
                    bhkWorld_RemoveConstraint(world, endpoint->constraint->constraint);
                if (endpoint->source)
                    bhkRigidBody_RemoveConstraintFromArray(endpoint->source, endpoint->constraint);
                endpoint->constraint = nullptr;
            }
            *endpoint = {};
        }
        if (lease && planck && planckClient)
            planck->ReleaseLease(planckClient, lease);
        state.flags &= ~(HIGGS_SUPPORT_TARGET_VALID | HIGGS_SUPPORT_NATIVE_APPLIED | HIGGS_SUPPORT_CONSTRAINT_ATTACHED);
        return HIGGS_HAND_OK;
    }
    auto player = *g_thePlayer;
    if (!player || !player->parentCell)
        return HIGGS_HAND_NOT_READY;
    NiPointer<bhkWorld> world = GetHavokWorldFromCell(player->parentCell);
    if (!world || !world->world)
        return HIGGS_HAND_NOT_READY;
    {
        std::lock_guard lock(endpointMutex);
        auto endpoint = Find(state.requestId);
        if (!endpoint)
        {
            for (auto &slot : endpoints)
                if (!slot.id)
                {
                    endpoint = &slot;
                    break;
                }
            if (!endpoint)
                return HIGGS_HAND_CAPACITY;
            endpoint->id = state.requestId;
            endpoint->world = world;
        }
        if (endpoint->world != world || endpoint->invalid)
            return HIGGS_HAND_STALE;
    }
    if (IsPlanck(state.request.kind))
    {
        if (!planck || !interop || !planckClient)
            return HIGGS_HAND_UNSUPPORTED;
        PlanckToken lease = 0;
        {
            std::lock_guard lock(endpointMutex);
            lease = Find(state.requestId)->lease;
        }
        if (!lease)
        {
            PlanckEndpointQuery query{};
            query.size = sizeof(query);
            query.body = Body(state.request.providerBody);
            query.localPoint = {state.request.targetLocalPoint[0], state.request.targetLocalPoint[1],
                                state.request.targetLocalPoint[2]};
            PlanckEndpoint acquired{};
            acquired.size = sizeof(acquired);
            if (planck->AcquireAttachmentEndpoint(planckClient, &query, &acquired) != PLANCK_OK)
                return HIGGS_HAND_STALE;
            {
                std::lock_guard lock(endpointMutex);
                auto endpoint = Find(state.requestId);
                if (!endpoint)
                {
                    planck->ReleaseLease(planckClient, acquired.lease);
                    return HIGGS_HAND_STALE;
                }
                endpoint->lease = acquired.lease;
                lease = acquired.lease;
            }
        }
        // No engine/world lock is held before PLANCK takes graph -> world.
        Visit visit{state.requestId, &state};
        const auto status = interop->WithAttachmentEndpoint(planckClient, lease, WithEndpoint, &visit);
        if (status == PLANCK_BUSY || status == PLANCK_REENTRANT)
            return HIGGS_HAND_PENDING;
        return status == PLANCK_OK ? visit.status : HIGGS_HAND_STALE;
    }
    if (state.request.kind == HIGGS_SUPPORT_PLAYER_WEAPON)
        return HIGGS_HAND_UNSUPPORTED;
    auto form = LookupFormByID(state.request.referenceFormId);
    auto reference = form ? DYNAMIC_CAST(form, TESForm, TESObjectREFR) : nullptr;
    if (!reference || DYNAMIC_CAST(reference, TESObjectREFR, Actor))
        return HIGGS_HAND_STALE;
    auto model = reference->GetNiNode();
    if (!model)
        return HIGGS_HAND_NOT_READY;
    BSWriteLocker worldLock(&world->worldLock);
    auto wrapper = GetFirstRigidBody(model);
    if (!wrapper || !wrapper->hkBody || wrapper->GetHavokWorld_1() != world->world)
        return HIGGS_HAND_NOT_READY;
    std::lock_guard lock(endpointMutex);
    auto endpoint = Find(state.requestId);
    if (!endpoint)
        return HIGGS_HAND_STALE;
    if (endpoint->wrapper &&
        (endpoint->reference != reference || endpoint->model != model || endpoint->wrapper != wrapper))
        return HIGGS_HAND_STALE;
    endpoint->reference = reference;
    endpoint->model = model;
    endpoint->wrapper = wrapper;
    return ApplyLocked(*endpoint, state, wrapper->hkBody, 0);
}
} // namespace
void InitializeNativeSupport(void *messaging, uint32_t handle)
{
    struct Message
    {
        void *(*GetApiFunction)(unsigned) = nullptr;
    } message;
    auto api = static_cast<SKSEMessagingInterface *>(messaging);
    if (api)
    {
        api->Dispatch(handle, 0x92F38745, &message, sizeof(message), "PLANCK");
        if (message.GetApiFunction)
        {
            auto observed = static_cast<PlanckSynergyAPI *>(message.GetApiFunction(2));
            auto borrowed = static_cast<PlanckPhysicalInteropAPI *>(message.GetApiFunction(3));
            if (observed && observed->size >= sizeof(*observed) && observed->version == 2 &&
                (observed->capabilities & PLANCK_CAP_ENDPOINTS) && borrowed && borrowed->size >= sizeof(*borrowed) &&
                borrowed->version == 1 && borrowed->WithAttachmentEndpoint)
            {
                PlanckClientIdentity identity{};
                identity.size = sizeof(identity);
                const char name[] = "HIGGS physical support";
                std::memcpy(identity.id, name, sizeof(name));
                PlanckToken client = 0;
                if (observed->RegisterClient(&identity, &client) == PLANCK_OK)
                {
                    PlanckToken subscription = 0;
                    if (observed->Subscribe(client, PLANCK_ENDPOINT_INVALIDATED | PLANCK_RESET, Invalidated, nullptr,
                                            &subscription) == PLANCK_OK)
                    {
                        planck = observed;
                        interop = borrowed;
                        planckClient = client;
                        planckSubscription = subscription;
                    }
                    else
                        observed->ReleaseClient(client);
                }
            }
        }
    }
    const uint64_t ordinary =
        (UINT64_C(1) << (HIGGS_SUPPORT_MOVING_WORLD - 1)) | (UINT64_C(1) << (HIGGS_CONSTRAINT_HELD_TO_WORLD - 1));
    const uint64_t npc = planckClient ? (UINT64_C(1) << (HIGGS_SUPPORT_PLANCK_HAND - 1)) |
                                            (UINT64_C(1) << (HIGGS_CONSTRAINT_HELD_TO_PLANCK - 1))
                                      : 0;
    Supports().Install(Driver, nullptr, ordinary | npc);
}
} // namespace HiggsHands
