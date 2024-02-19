#pragma once

#include <numeric>
#include <array>
#include <deque>
#include <mutex>

#include "skse64/InternalVR.h"

#include "RE/havok.h"
#include "physics.h"
#include "utils.h"
#include "haptics.h"
#include "finger_animator.h"
#include "config.h"

#include <Physics/Collide/Shape/Convex/Box/hkpBoxShape.h>


struct Hand
{
    struct SelectedObject
    {
        NiPointer<bhkRigidBody> rigidBody;
        hkpCollidable *collidable = nullptr; // ptr to collidable owned by rigidBody
        TESForm *hitForm;
        BaseExtraList *hitExtraList;
        UInt32 hitFormCount;
        NiPointer<NiAVObject> shaderNode;
        NiPointer<NiAVObject> hitNode;
        std::unordered_map<bhkRigidBody *, hkInt16> savedContactPointCallbackDelays;
        std::unordered_map<bhkRigidBody *, NiPoint3> savedInverseInertias;
        std::deque<NiPoint3> linearVelocities{ 5, NiPoint3() };
        NiPoint3 point; // in meters (havok), the last point that was selected by the collision checks
        UInt32 handle = 0;
        float totalMass = 0.f;
        UInt32 collisionGroup = 0;
        hkpMotion::MotionType savedMotionType = hkpMotion::MotionType::MOTION_INVALID;
        hkInt8 savedQuality = HK_COLLIDABLE_QUALITY_INVALID;
        UInt8 savedRigidBodyFlags;
        bool isActor = false;
        bool isBook = false;
        bool isDisconnected = false;
        bool isImpactedProjectile = false;
    };

    struct PulledObject
    {
        NiPointer<bhkRigidBody> rigidBody;
        UInt32 handle = 0;
        hkHalf savedAngularDamping;
        UInt32 collisionGroup = 0;
    };

    struct TwoHandedState
    {
        enum class AngleState : UInt8 {
            None,
            CrossPi,
            CrossNegativePi
        };

        NiTransform weaponOffsetNodeLocalTransform;
        NiTransform collisionOffsetNodeLocalTransform;
        NiTransform wandNodeLocalTransform;
        NiTransform handToWand;
        NiTransform handToWeapon;
        NiTransform prevWeaponTransform;
        NiTransform prevWeaponTransformRoomspace;
        TESObjectWEAP *weapon;
        float prevFrameTwistAngle360 = 0;
        AngleState angleState = AngleState::None;
    };

    struct ControllerTrackingData
    {
        std::deque<NiPoint3> linearVelocities{ 5, NiPoint3() };
        std::deque<NiPoint3> angularVelocities{ 5, NiPoint3() };
        NiPoint3 avgVelocity;
        float avgSpeed;

        void RecomputeAverageVelocity()
        {
            avgVelocity = std::accumulate(linearVelocities.begin(), linearVelocities.end(), NiPoint3()) / linearVelocities.size();
        }

        void RecomputeAverageSpeed()
        {
            float speed = 0;
            for (NiPoint3 &velocity : linearVelocities) {
                speed += VectorLength(velocity);
            }
            speed /= std::size(linearVelocities);
            avgSpeed = speed;
        }

        void Recompute()
        {
            RecomputeAverageVelocity();
            RecomputeAverageSpeed();
        }
    };

    enum class State : UInt8
    {
        Idle, // not pointing at anything meaningful
        SelectedFar, // pointing at something meaningful that isn't close
        SelectedClose, // pointing at something that's next to the hand
        SelectionLocked, // player has locked in their selection, i.e. is holding the button
        PreGrabItem, // player wants to grab an object that isn't loaded yet
        PrePullItem, // player wants to pull a piece of armor off, wait for it to spawn
        Pulled, // first few frames when a player is pulling the object towards them
        HeldInit, // held object is moving towards hand
        Held, // player is holding the object in their hand
        HeldBody, // player is holding a body / other constrained object
        GrabFromOtherHand, // wait after requesting the other hand to drop the object so that we can grab it
        GrabExternal, // want to grab an object that we didn't have selected already
        LootOtherHand, // want to loot from what the other hand is holding
        SelectedTwoHand,
        HeldTwoHanded,
    };

    enum class DampingState : UInt8
    {
        Undamped,
        PreDamp,
        Damped,
        TryLeaveDamped
    };

    enum class InputState : UInt8
    {
        Idle,
        Leeway,
        Block,
        Force
    };

    Hand(bool isLeft, BSFixedString name, BSFixedString handNodeName, BSFixedString wandNodeName, BSFixedString grabNodeOnObjectName, BSFixedString fingerNodeNames[5][3], NiPoint3 palmPosHandspace, NiPoint3 rolloverOffset, bool delayGripInput) :
        isLeft(isLeft),
        name(name),
        handNodeName(handNodeName),
        wandNodeName(wandNodeName),
        grabNodeOnObjectName(grabNodeOnObjectName),
        palmPosHandspace(palmPosHandspace),
        rolloverOffset(rolloverOffset),
        delayGripInput(delayGripInput),
        haptics(isLeft ? BSVRInterface::BSControllerHand::kControllerHand_Left : BSVRInterface::BSControllerHand::kControllerHand_Right),
        fingerAnimator(isLeft, fingerNodeNames)
    {
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 3; j++) {
                this->fingerNodeNames[i][j] = fingerNodeNames[i][j];
            }
        }
    };

    ~Hand() = delete; // Hacky way to prevent trying to free NiPointers when the game quits and memory is fucked

    void Update(Hand &other, bhkWorld *world);
    void PostUpdate(Hand &other, bhkWorld *world);
    void ControllerStateUpdate(uint32_t unControllerDeviceIndex, vr_src::VRControllerState001_t *pControllerState);

    void PlaySelectionEffect(UInt32 objHandle, NiAVObject *node);
    void StopSelectionEffect(UInt32 objHandle, NiAVObject *node);
    void ResetNearbyDamping();
    void StartNearbyDamping(bhkWorld &world);
    bool FindCloseObject(bhkWorld *world, const Hand &other, const NiPoint3 &hkPalmNodePos, const NiPoint3 &castDirection, const bhkSimpleShapePhantom *sphere, bool isTwoHandedOffhand,
        NiPointer<TESObjectREFR> &closestObj, NiPointer<bhkRigidBody> &closestRigidBody, hkVector4 &closestPoint);
    bool FindFarObject(bhkWorld *world, const Hand &other, const NiPoint3 &hkPalmNodePos, const NiPoint3 &castDirection, const NiPoint3 &hkHmdPos, const NiPoint3 &hmdForward, const bhkSimpleShapePhantom *sphere,
        NiPointer<TESObjectREFR> &closestObj, NiPointer<bhkRigidBody> &closestRigidBody, hkVector4 &closestPoint);
    bool FindOtherWeapon(bhkWorld *world, const Hand &other, const NiPoint3 &startPos, const NiPoint3 &castDirection, const bhkSimpleShapePhantom *sphere, hkVector4 &hitPoint);
    void CreateHandCollision(bhkWorld *world);
    void RemoveHandCollision(bhkWorld *world);
    void RemoveHandCollisionFromCurrentWorld();
    void UpdateHandCollision(bhkWorld *world);
    hkTransform ComputeHandCollisionTransform(bool isBeast, const NiTransform * a_handTransform = nullptr);
    hkTransform ComputeWeaponCollisionTransform(bhkRigidBody *existingWeaponCollision);
    void CreateWeaponCollision(bhkWorld *world);
    void RemoveWeaponCollision(bhkWorld *world);
    void RemoveWeaponCollisionFromCurrentWorld();
    void UpdateWeaponCollision();
    std::optional<NiTransform> GetAttachTransform(const TESForm *baseForm);
    std::optional<NiTransform> ComputeInitialObjectTransform(TESObjectREFR *refr, const NiAVObject *handNode, NiAVObject *grabbedNode);
    std::optional<NiTransform> GetInitialObjectTransformBasedOnGrabNodes(const NiAVObject *handNode, NiAVObject *grabbedNode);
    bool ShouldUsePhysicsBasedGrab(TESObjectREFR *refr, bhkRigidBody *rigidBody);
    NiPointer<bhkRigidBody> Hand::GetRigidBodyToGrabBasedOnGeometry(const Hand &other, TESObjectREFR *selectedObj, const NiPoint3 &palmPos, const NiPoint3 &palmDirection, const std::optional<NiTransform> &initialTransform, NiAVObject *handNode);
    void TransitionHeld(Hand &other, bhkWorld &world, const NiPoint3 &hkPalmNodePos, const NiPoint3 &castDirection, const NiPoint3 &closestPoint, float havokWorldScale, const NiAVObject *handNode, float handSize, TESObjectREFR *selectedObj,
        const std::optional<NiTransform> &initialTransform = std::nullopt, bool warpToHand = false, bool reuseTriangles = false, bool playSound = true);
    void TransitionHeldTwoHanded(Hand &other, bhkWorld &world, const NiPoint3 &hkPalmPos, const NiPoint3 &palmDirection, const NiPoint3 &closestPoint,
        float havokWorldScale, const NiTransform &handTransform, float handSize, NiAVObject *weaponNode, TESObjectWEAP *otherHandWeapon);
    void TransitionPreGrab(TESObjectREFR *selectedObj, bool isExternal = false);
    bool TransitionGrabExternal(TESObjectREFR *refr);
    void GrabExternalObject(Hand &other, bhkWorld &world, TESObjectREFR *selectedObj, NiNode *objRoot, NiAVObject *collidableNode, NiAVObject *handNode, float handSize, bhkSimpleShapePhantom *sphere, const NiPoint3 &hkPalmNodePos, const NiPoint3 &palmVector, float havokWorldScale);
    void SetPulledDuration(const NiPoint3 &hkPalmNodePos, const NiPoint3 &objPoint);
    NiPointer<NiAVObject> GetFirstPersonHandNode();
    NiPointer<NiAVObject> GetThirdPersonHandNode();
    NiPointer<NiAVObject> GetWeaponOffsetNode(TESObjectWEAP *weapon);
    NiPointer<NiAVObject> GetWeaponCollisionOffsetNode(TESObjectWEAP *weapon);
    NiPointer<NiAVObject> GetWeaponNode(bool thirdPerson);
    inline NiPointer<NiAVObject> GetWandNode() { return isLeft ? (*g_thePlayer)->unk3F0[PlayerCharacter::Node::kNode_LeftWandNode] : (*g_thePlayer)->unk3F0[PlayerCharacter::Node::kNode_RightWandNode]; }
    inline NiPointer<NiAVObject> GetMagicOffsetNode() { return (*g_leftHandedMode != isLeft) ? (*g_thePlayer)->unk3F0[PlayerCharacter::Node::kNode_SecondaryMagicOffsetNode] : (*g_thePlayer)->unk3F0[PlayerCharacter::Node::kNode_PrimaryMagicOffsetNode]; }
    inline NiPointer<NiAVObject> GetMagicAimNode() { return (*g_leftHandedMode != isLeft) ? (*g_thePlayer)->unk3F0[PlayerCharacter::Node::kNode_SecondaryMagicAimNode] : (*g_thePlayer)->unk3F0[PlayerCharacter::Node::kNode_PrimaryMagicAimNode]; }
    NiPointer<NiAVObject> GetMagicNode(bool thirdPerson);
    float GetHandSize();
    Hand & GetOtherHand();
    void UpdateHandTransform(NiTransform &worldTransform);
    NiTransform GetGrabTransform();
    void SetGrabTransform(const NiTransform &transform);
    inline NiPoint3 GetPalmPositionWS(const NiTransform &handTransform) { return handTransform * palmPosHandspace; }
    NiPoint3 GetPalmVectorWS(NiMatrix33 &handRotation);
    NiPoint3 GetPointingVectorWS(NiMatrix33 &handRotation);
    NiPoint3 GetHandVelocity();
    bool IsObjectDepositable(TESObjectREFR *refr, NiAVObject *hmdNode, const NiPoint3 &handPos) const;
    bool IsObjectConsumable(TESObjectREFR *refr, NiAVObject *hmdNode, const NiPoint3 &handPos) const;
    bool IsTwoHanding() const;
    UInt32 SpawnEquippedSelectedObject(TESObjectREFR *selectedObj, float zOffsetWhenNotDisconnected);
    bool ShouldDisplayRollover(Hand &other);
    bool IsObjectPullable();
    bool HasExclusiveObject() const;
    bool IsInGrabbableState() const;
    bool CanHoldObject() const;
    bool CanTwoHand() const;
    bool CanGrabObject() const;
    bool HasHeldObject() const;
    bool CanOtherGrab() const;
    bool HasHeldKeyframed() const;
    bool HasHeldConstrained() const;
    bool HasIgnorableCollision() const;
    bool ShouldIgnoreCollisionGroup(UInt32 collisionGroup) const;
    bool GetActivateText(std::string &str);
    bool GetActivateButton(std::string &str);
    void SetupRollover(NiAVObject *rolloverNode);
    void SetupSelectionBeam(NiNode *spellOrigin);
    float GetEffectiveHeldMass();
    void Select(TESObjectREFR *obj);
    void Deselect();
    void PostVrikUpdate();
    void LateMainThreadUpdate();
    void PreUpdateHandsUpdate();
    void EndPull();
    void PlayPhysicsSound(const NiPoint3 &location, bool loud = false);
    void TriggerCollisionHaptics(float mass, float speed);
    bool CanHoldBasedOnWeapon() const;

    static const int equippedWeaponSlotBase = 32; // First biped slot to have equipped weapons

    HapticsManager haptics;
    FingerAnimator fingerAnimator;

    NiPointer<bhkRigidBody> handBody = nullptr;
    hkpCollidable *handCollidable = nullptr; // read only
    bool wasBeastWhenHandCollisionCreated = false;

    NiPointer<bhkRigidBody> weaponBody = nullptr; // Owned by us - this is our weapon collision
    hkpCollidable *weaponCollidable = nullptr; // read only
    const bhkShape *clonedFromWeaponShape = nullptr; // the shape of the collision object we cloned to create ours
    float weaponBodyHandSize = 1.f;

    const bool isLeft = false;
    const bool delayGripInput = false;
    const NiPoint3 palmPosHandspace{};
    BSFixedString fingerNodeNames[5][3]{}; // 5 fingers, 3 joints
    BSFixedString name{}; // Used for logging
    BSFixedString handNodeName{};
    BSFixedString wandNodeName{};
    BSFixedString grabNodeOnObjectName{};

    std::mutex deselectLock{};

    NiPoint3 rolloverOffset{};
    NiMatrix33 rolloverRotation{};
    float rolloverScale = 10.f;

    UInt16 playerCollisionGroup = 0;

    TESEffectShader *itemSelectedShader = nullptr;
    TESEffectShader *itemSelectedShaderOffLimits = nullptr;

    ControllerTrackingData controllerData{};
    std::deque<NiPoint3> playerVelocitiesWorldspace{ 5, NiPoint3() }; // previous n player velocities in skyrim worldspace
    NiPoint3 avgPlayerVelocityWorldspace{};
    float avgPlayerSpeedWorldspace = 0.f;

    std::vector<NiPointer<bhkRigidBody>> nearbyBodies{}; // This only exists to hold the NiPointers
    std::unordered_map<bhkRigidBody *, std::pair<hkHalf, hkHalf>> nearbyBodyMap{};

    SelectedObject selectedObject{};
    PulledObject pulledObject{};
    TwoHandedState twoHandedState{};

    NiPoint3 pulledPointOffset{}; // Offset from the center of mass of the point we're pulling on the pulled object
    NiPoint3 pullTarget{};

    NiPointer<bhkConstraint> grabConstraint = nullptr;
    bool fadeInGrabConstraint = false;

    NiTransform desiredNodeTransformHandSpace{};

    NiTransform prevHandTransformRoomspace{};

    NiTransform handTransform{};
    NiTransform clavicleTransform{};
    NiTransform adjustedHandTransform{};
    NiTransform thirdPersonHandToWeaponTransform{};

    NiTransform fpAnimHandTransform{};
    NiTransform fpAnimWeaponTransform{};

    NiPoint3 prevPlayerPosWorldspace{};
    NiPoint3 playerDeltaPos{};
    NiPoint3 prevPlayerVelocityWorldspace{};
    NiPoint3 playerAcceleration{};

    std::vector<TriangleData> triangles{}; // tris are in worldspace
    NiTransform previousTriangleAdjustment{};

    std::set<NiPointer<bhkRigidBody>> connectedRigidBodies{};
    std::unordered_set<const bhkRigidBody *> playerPositionUpdatedRigidBodies{};

    std::deque<float> handDeviations{ 5, 0.f };
    bool isSneaking = false;
    double sneakUnsneakTime = 0;

    bool idleDesired = false;

    bool isExternalGrab = false;
    bool disableDropEvents = false;

    bool externalGrabRequested = false;
    NiPointer<TESObjectREFR> externalGrabRequestedObject = nullptr;

    double lastSelectedTime = 0; // Timestamp of the last time we were pointing at something valid
    double grabRequestedTime = 0; // Timestamp when the trigger was pressed
    double rolloverDisplayTime = 0; // Timestamp when we performed the last action that warrants showing the rollover text
    double grabbedTime = 0; // Timestamp when the currently grabbed object (if there is one) was grabbed
    double droppedTime = 0;
    double pullDuration = 0;
    double pulledExpireTime = 0; // Amount of time after pulling to wait before restoring original collision information
    double pulledTime = 0; // Timestamp when the last pulled object was pulled
    double heldTime = 0;
    double forceInputTime = 0;
    double preDampTime = 0;
    double tryLeaveDampedTime = 0;
    double weaponHitTime = 0;
    double rolloverAlphaSetTime = 0;
    double lastWasSnapTurningTime = 0;
    double startGrabLerpHandDuration = 0;

    double selectedTwoHandTime = 0;
    bool wasSelectedTwoHandLerping = false;

    float handSize = 1.f;

    float grabbedFingerValues[5];
    bool useAlternateThumbCurve = false;

    State state = State::Idle;
    State prevState = State::Idle;

    InputState inputState = InputState::Idle;
    bool inputTrigger = false;
    bool inputGrip = false;

    bool triggerDown = false; // Whether the trigger was down last frame
    bool gripDown = false;
    bool grabRequested = false; // True on rising edge of trigger press
    bool releaseRequested = false; // True on falling edge of trigger press
    bool wasObjectGrabbed = false;
};

extern Hand *g_rightHand;
extern Hand *g_leftHand;
