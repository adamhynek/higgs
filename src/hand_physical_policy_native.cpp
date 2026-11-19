#include "RE/offsets.h"
#include "hand.h"
#include "hand_broker.h"
#include "hand_physical_policy.h"
#include "main.h"
#include "skse64/GameRTTI.h"
#include <algorithm>
#include <cmath>

namespace HiggsHands
{
namespace
{
struct NativePolicy
{
    uint64_t id = 0, revision = 0, generation = 0, epoch = 0;
    uint32_t fields = 0;
    NiPointer<bhkWorld> world;
    NiPointer<TESObjectREFR> reference;
    NiPointer<NiNode> model;
    NiPointer<NiAVObject> collisionNode, controller;
    NiPointer<bhkRigidBody> wrapper;
    RE::hkRefPtr<hkpRigidBody> body;
    RE::hkRefPtr<hkpShape> shape;
    hkpMotion *motion = nullptr;
    uint32_t motionType = 0, originalCull = 0;
    HIGGS_PhysicalPolicyValues baseline{}, last{};
};
std::array<NativePolicy, 64> nativePolicies{};
uint64_t nextBodyGeneration = 1;
NativePolicy *Find(uint64_t id)
{
    for (auto &policy : nativePolicies)
        if (policy.id == id)
            return &policy;
    return nullptr;
}
bool Near(float a, float b)
{
    return std::isfinite(a) && std::isfinite(b) &&
           std::fabs(a - b) <= 1.e-4f * (std::max)(1.f, (std::max)(std::fabs(a), std::fabs(b)));
}
bool OwnedEqual(float a, float b)
{
    return std::isfinite(a) && std::isfinite(b) && a == b;
}
bool OwnedMatch(const HIGGS_PhysicalPolicyValues &a, const HIGGS_PhysicalPolicyValues &b, uint32_t fields)
{
    if (fields & 3)
    {
        if (!OwnedEqual(a.mass, b.mass))
            return false;
        for (size_t i = 0; i < 9; ++i)
            if (!OwnedEqual(a.inertia[i], b.inertia[i]))
                return false;
    }
    if (fields & 4)
        for (size_t i = 0; i < 3; ++i)
            if (!OwnedEqual(a.centerOfMass[i], b.centerOfMass[i]))
                return false;
    return true;
}
bool Match(const HIGGS_PhysicalPolicyValues &a, const HIGGS_PhysicalPolicyValues &b, uint32_t fields)
{
    if (fields & 3)
    {
        if (!Near(a.mass, b.mass))
            return false;
        for (size_t i = 0; i < 9; ++i)
            if (!Near(a.inertia[i], b.inertia[i]))
                return false;
    }
    if (fields & 4)
        for (size_t i = 0; i < 3; ++i)
            if (!Near(a.centerOfMass[i], b.centerOfMass[i]))
                return false;
    return true;
}
bool Read(hkpMotion *motion, HIGGS_PhysicalPolicyValues &value)
{
    const float inverse = motion->m_inertiaAndMassInv(3);
    if (!std::isfinite(inverse) || inverse <= 0)
        return false;
    value.mass = 1.f / inverse;
    if (!std::isfinite(value.mass) || value.mass < .0001f || value.mass > 1.e6f)
        return false;
    hkMatrix3 inertia;
    motion->getInertiaLocal(inertia);
    for (size_t r = 0; r < 3; ++r)
        for (size_t c = 0; c < 3; ++c)
        {
            value.inertia[r * 3 + c] = inertia(static_cast<int>(r), static_cast<int>(c));
            if (!std::isfinite(value.inertia[r * 3 + c]))
                return false;
        }
    for (size_t i = 0; i < 3; ++i)
    {
        value.centerOfMass[i] = motion->getCenterOfMassLocal()(static_cast<int>(i));
        if (!std::isfinite(value.centerOfMass[i]))
            return false;
    }
    const auto &m = value.inertia;
    const double determinant = double(m[0]) * (double(m[4]) * m[8] - double(m[5]) * m[7]) -
                               double(m[1]) * (double(m[3]) * m[8] - double(m[5]) * m[6]) +
                               double(m[2]) * (double(m[3]) * m[7] - double(m[4]) * m[6]);
    return m[0] > 0 && m[4] > 0 && m[8] > 0 && determinant > 0 && std::isfinite(determinant);
}
void Write(hkpMotion *motion, const HIGGS_PhysicalPolicyValues &value, uint32_t fields)
{
    const auto untouchedCenter = motion->getCenterOfMassLocal();
    if (fields & 3)
    {
        if (fields & HIGGS_PROPERTY_MASS)
            motion->setMass(value.mass);
        NiMatrix33 matrix;
        for (size_t r = 0; r < 3; ++r)
            for (size_t c = 0; c < 3; ++c)
                matrix.data[r][c] = value.inertia[r * 3 + c];
        hkMatrix3 inertia;
        NiMatrixToHkMatrix(matrix, inertia);
        motion->setInertiaLocal(inertia);
        if (!(fields & HIGGS_PROPERTY_CENTER_OF_MASS))
            motion->setCenterOfMassInLocal(untouchedCenter);
    }
    if (fields & 4)
        motion->setCenterOfMassInLocal(
            NiPointToHkVector({value.centerOfMass[0], value.centerOfMass[1], value.centerOfMass[2]}));
}
NiPointer<NiAVObject> Controller(uint32_t hand)
{
    auto player = *g_thePlayer;
    if (!player || !player->GetNiNode())
        return nullptr;
    return player->unk3F0[hand == HIGGS_HAND_LEFT ? PlayerCharacter::Node::kNode_LeftControllerNode
                                                  : PlayerCharacter::Node::kNode_RightControllerNode];
}
bool Lifetime(const NativePolicy &policy)
{
    return policy.body && policy.wrapper && policy.wrapper->hkBody == policy.body &&
           policy.body->getWorld() == policy.world->world && policy.body->getCollidable()->getShape() == policy.shape &&
           policy.body->getMotion() == policy.motion && policy.motion->m_type == policy.motionType &&
           policy.reference && LookupFormByID(policy.reference->formID) == policy.reference.operator->() &&
           policy.reference->GetNiNode() == policy.model && policy.collisionNode &&
           GetRigidBody(policy.collisionNode) == policy.wrapper;
}
HIGGS_HandStatus Driver(HIGGS_PhysicalPolicyState &state, bool cleanup, void *)
{
    const auto &request = state.request;
    if (request.kind == HIGGS_POLICY_GRAB_MODE ||
        !(request.kind == HIGGS_POLICY_PROPERTIES || (request.presentation & HIGGS_PRESENTATION_CULL_CONTROLLER)))
    {
        if (cleanup)
            state.flags |= HIGGS_POLICY_RESTORED;
        return HIGGS_HAND_OK;
    }
    auto policy = Find(state.policy);
    if (cleanup)
    {
        if (!policy)
            return HIGGS_HAND_OK;
        if (request.kind == HIGGS_POLICY_PROPERTIES)
        {
            NiPointer<bhkWorld> cleanupWorld = policy->world;
            BSWriteLocker lock(&cleanupWorld->worldLock);
            HIGGS_PhysicalPolicyValues current{};
            if (Lifetime(*policy) && Read(policy->motion, current))
            {
                auto desired = current;
                uint32_t candidates = 0, writeFields = 0;
                if (request.fields & 3)
                {
                    if (OwnedEqual(current.mass, policy->last.mass))
                    {
                        desired.mass = policy->baseline.mass;
                        candidates |= HIGGS_PROPERTY_MASS;
                    }
                    else
                        state.flags |= HIGGS_POLICY_EXTERNAL_WRITE;
                    bool inertiaMatches = true;
                    for (size_t i = 0; i < 9; ++i)
                        inertiaMatches = inertiaMatches && OwnedEqual(current.inertia[i], policy->last.inertia[i]);
                    if (inertiaMatches)
                    {
                        for (size_t i = 0; i < 9; ++i)
                            desired.inertia[i] = policy->baseline.inertia[i];
                        candidates |= HIGGS_PROPERTY_INERTIA;
                    }
                    else
                        state.flags |= HIGGS_POLICY_EXTERNAL_WRITE;
                    if ((candidates & HIGGS_PROPERTY_MASS) && !OwnedEqual(current.mass, desired.mass))
                        writeFields |= HIGGS_PROPERTY_MASS;
                    if (candidates & HIGGS_PROPERTY_INERTIA)
                        for (size_t i = 0; i < 9; ++i)
                            if (!OwnedEqual(current.inertia[i], desired.inertia[i]))
                                writeFields |= HIGGS_PROPERTY_INERTIA;
                }
                if (request.fields & HIGGS_PROPERTY_CENTER_OF_MASS)
                {
                    bool centerMatches = true;
                    for (size_t i = 0; i < 3; ++i)
                        centerMatches =
                            centerMatches && OwnedEqual(current.centerOfMass[i], policy->last.centerOfMass[i]);
                    if (centerMatches)
                    {
                        for (size_t i = 0; i < 3; ++i)
                            desired.centerOfMass[i] = policy->baseline.centerOfMass[i];
                        candidates |= HIGGS_PROPERTY_CENTER_OF_MASS;
                        writeFields |= HIGGS_PROPERTY_CENTER_OF_MASS;
                    }
                    else
                        state.flags |= HIGGS_POLICY_EXTERNAL_WRITE;
                }
                if (writeFields)
                    Write(policy->motion, desired, writeFields);
                HIGGS_PhysicalPolicyValues restored{};
                uint32_t completed = 0;
                if (Read(policy->motion, restored))
                {
                    state.effective = restored;
                    if ((candidates & HIGGS_PROPERTY_MASS) && Near(restored.mass, policy->baseline.mass))
                    {
                        completed |= HIGGS_PROPERTY_MASS;
                        state.flags |= HIGGS_POLICY_RESTORED_MASS;
                    }
                    bool inertiaMatches = true;
                    for (size_t i = 0; i < 9; ++i)
                        inertiaMatches = inertiaMatches && Near(restored.inertia[i], policy->baseline.inertia[i]);
                    if ((candidates & HIGGS_PROPERTY_INERTIA) && inertiaMatches)
                    {
                        completed |= HIGGS_PROPERTY_INERTIA;
                        state.flags |= HIGGS_POLICY_RESTORED_INERTIA;
                    }
                    bool centerMatches = true;
                    for (size_t i = 0; i < 3; ++i)
                        centerMatches =
                            centerMatches && Near(restored.centerOfMass[i], policy->baseline.centerOfMass[i]);
                    if ((candidates & HIGGS_PROPERTY_CENTER_OF_MASS) && centerMatches)
                    {
                        completed |= HIGGS_PROPERTY_CENTER_OF_MASS;
                        state.flags |= HIGGS_POLICY_RESTORED_COM;
                    }
                }
                if ((completed & request.fields) == request.fields)
                    state.flags |= HIGGS_POLICY_RESTORED;
                else
                    state.flags |= HIGGS_POLICY_EXTERNAL_WRITE;
            }
            else
                state.flags |= HIGGS_POLICY_EXTERNAL_WRITE;
            *policy = {};
        }
        else
        {
            if (Controller(request.physicalHand) == policy->controller && policy->controller &&
                (policy->controller->m_flags & 1) == 1)
            {
                policy->controller->m_flags = (policy->controller->m_flags & ~1u) | policy->originalCull;
                state.flags |= HIGGS_POLICY_RESTORED;
            }
            else
                state.flags |= HIGGS_POLICY_EXTERNAL_WRITE;
            *policy = {};
        }
        return HIGGS_HAND_OK;
    }
    if (request.kind == HIGGS_POLICY_PRESENTATION)
    {
        auto node = Controller(request.physicalHand);
        if (!node)
            return HIGGS_HAND_NOT_READY;
        if (!policy)
        {
            for (const auto &other : nativePolicies)
                if (other.id && other.controller == node)
                    return other.epoch == state.epoch ? HIGGS_HAND_CONFLICT : HIGGS_HAND_PENDING;
            for (auto &slot : nativePolicies)
                if (!slot.id)
                {
                    policy = &slot;
                    break;
                }
            if (!policy)
                return HIGGS_HAND_CAPACITY;
            policy->id = state.policy;
            policy->epoch = state.epoch;
            policy->controller = node;
            policy->originalCull = node->m_flags & 1;
        }
        else if (policy->controller != node || (node->m_flags & 1) != 1)
        {
            state.flags |= HIGGS_POLICY_EXTERNAL_WRITE;
            return HIGGS_HAND_CONFLICT;
        }
        node->m_flags |= 1;
        policy->revision = state.revision;
        state.appliedFields |= HIGGS_PRESENTATION_CULL_CONTROLLER;
        return HIGGS_HAND_OK;
    }
    auto hand = request.physicalHand == HIGGS_HAND_LEFT ? g_leftHand : g_rightHand;
    auto player = *g_thePlayer;
    if (!hand || !player || !player->parentCell || !hand->HasHeldConstrained() || hand->selectedObject.isActor ||
        !hand->selectedObject.rigidBody)
        return HIGGS_HAND_UNSUPPORTED;
    NiPointer<TESObjectREFR> reference;
    if (!LookupREFRByHandle(hand->selectedObject.handle, reference) || !reference ||
        reference->formID != request.item.worldRefId)
        return HIGGS_HAND_STALE;
    NiPointer<bhkWorld> world = GetHavokWorldFromCell(player->parentCell);
    if (!world)
        return HIGGS_HAND_NOT_READY;
    BSWriteLocker lock(&world->worldLock);
    auto wrapper = hand->selectedObject.rigidBody;
    auto body = wrapper->hkBody;
    if (!body || body->getWorld() != world->world || !IsMoveableEntity(body) || !body->getCollidable()->getShape())
        return HIGGS_HAND_UNSUPPORTED;
    if (!policy)
    {
        const auto resources = [](uint32_t fields) { return (fields & 3 ? 3u : 0u) | (fields & 4); };
        for (const auto &other : nativePolicies)
            if (other.id && other.body == body && (resources(other.fields) & resources(request.fields)))
                return other.epoch == state.epoch ? HIGGS_HAND_CONFLICT : HIGGS_HAND_PENDING;
        for (auto &slot : nativePolicies)
            if (!slot.id)
            {
                policy = &slot;
                break;
            }
        if (!policy)
            return HIGGS_HAND_CAPACITY;
        NativePolicy candidate{};
        candidate.id = state.policy;
        candidate.epoch = state.epoch;
        candidate.fields = request.fields;
        candidate.world = world;
        candidate.reference = reference;
        candidate.model = reference->GetNiNode();
        candidate.wrapper = wrapper;
        candidate.body = body;
        candidate.shape = const_cast<hkpShape *>(body->getCollidable()->getShape());
        candidate.collisionNode = GetNodeFromCollidable(body->getCollidable());
        candidate.motion = body->getMotion();
        candidate.motionType = candidate.motion->m_type;
        if (!Lifetime(candidate) || !Read(candidate.motion, candidate.baseline))
            return HIGGS_HAND_NOT_READY;
        candidate.last = candidate.baseline;
        candidate.generation = nextBodyGeneration++;
        *policy = candidate;
    }
    else if (policy->world != world || !Lifetime(*policy) || policy->body != body)
        return HIGGS_HAND_STALE;
    HIGGS_PhysicalPolicyValues current{};
    if (!Read(policy->motion, current))
        return HIGGS_HAND_NOT_READY;
    if (!OwnedMatch(current, policy->last, request.fields))
    {
        state.flags |= HIGGS_POLICY_EXTERNAL_WRITE;
        return HIGGS_HAND_CONFLICT;
    }
    if (policy->revision != state.revision)
    {
        auto desired = policy->baseline;
        if (request.fields & HIGGS_PROPERTY_MASS)
            desired.mass *= request.massMultiplier;
        if (request.fields & HIGGS_PROPERTY_INERTIA)
            for (float &value : desired.inertia)
                value *= request.inertiaMultiplier;
        if (request.fields & HIGGS_PROPERTY_CENTER_OF_MASS)
            for (size_t i = 0; i < 3; ++i)
                desired.centerOfMass[i] += request.centerOfMassOffset[i];
        if (!std::isfinite(desired.mass) || desired.mass < .0001f || desired.mass > 1.e6f)
            return HIGGS_HAND_INVALID_ARGUMENT;
        for (float value : desired.inertia)
            if (!std::isfinite(value) || std::fabs(value) > 1.e12f)
                return HIGGS_HAND_INVALID_ARGUMENT;
        Write(policy->motion, desired, request.fields);
        if (!Read(policy->motion, policy->last) || !Match(policy->last, desired, request.fields))
            return HIGGS_HAND_NOT_READY;
        policy->revision = state.revision;
    }
    state.bodyGeneration = policy->generation;
    state.original = policy->baseline;
    state.effective = policy->last;
    state.appliedFields = request.fields;
    return HIGGS_HAND_OK;
}
} // namespace
void MaintainNativePhysicalPolicies()
{
    static bool installed = false;
    if (!installed)
    {
        PhysicalPolicies().Install(Driver, nullptr,
                                   HIGGS_POLICY_CAP_PROPERTIES | HIGGS_POLICY_CAP_GRAB_MODE |
                                       HIGGS_POLICY_CAP_FINGER_SUPPRESSION | HIGGS_POLICY_CAP_CONTROLLER_NODE_CULL);
        installed = true;
    }
    PhysicalPolicies().Drain(Now());
}
} // namespace HiggsHands
