#include <chrono>
#include <filesystem>

#include "config.h"
#include "math_utils.h"
#include "utils.h"


static inline double vlibGetSetting(const char * name) {
    Setting * setting = GetINISetting(name);
    double value;
    if (!setting)
        return -1;
    if (setting->GetDouble(&value))
        return value;
    return -1;
}


namespace Config {
    // Define extern options
    Options options;
    std::map<std::string, float*, std::less<>> floatMap{};
    std::map<std::string, double*, std::less<>> doubleMap{};
    std::map<std::string, int*, std::less<>> intMap{};
    std::map<std::string, bool*, std::less<>> boolMap{};
    bool g_registrationComplete = false;

    bool ReadFloat(const std::string &name, float &val, bool isImportant=true)
    {
        if (!GetConfigOptionFloat("Settings", name.c_str(), &val)) {
            if (isImportant) {
                _WARNING("Failed to read float config option: %s", name.c_str());
            }
            return false;
        }

        return true;
    }

    bool ReadDouble(const std::string &name, double &val)
    {
        if (!GetConfigOptionDouble("Settings", name.c_str(), &val)) {
            _WARNING("Failed to read double config option: %s", name.c_str());
            return false;
        }

        return true;
    }

    bool ReadBool(const std::string &name, bool &val)
    {
        if (!GetConfigOptionBool("Settings", name.c_str(), &val)) {
            _WARNING("Failed to read bool config option: %s", name.c_str());
            return false;
        }

        return true;
    }

    bool ReadInt(const std::string &name, int &val, bool isImportant=true)
    {
        if (!GetConfigOptionInt("Settings", name.c_str(), &val)) {
            if (isImportant) {
                _WARNING("Failed to read int config option: %s", name.c_str());
            }
            return false;
        }

        return true;
    }

    bool ReadString(const std::string &name, std::string &val)
    {
        std::string	data = GetConfigOption("Settings", name.c_str());
        if (data.empty()) {
            _WARNING("Failed to read str config option: %s", name.c_str());
            return false;
        }

        val = std::move(data);
        return true;
    }

    bool ReadVector(const std::string &name, NiPoint3 &vec)
    {
        if (!ReadFloat(name + "X", vec.x)) return false;
        if (!ReadFloat(name + "Y", vec.y)) return false;
        if (!ReadFloat(name + "Z", vec.z)) return false;

        return true;
    }

    bool ReadStringSet(const std::string &name, std::set<std::string, std::less<>> &val)
    {
        std::string	data = GetConfigOption("Settings", name.c_str());
        if (data.empty()) {
            _WARNING("Failed to read StringSet config option: %s", name.c_str());
            return false;
        }

        val = SplitStringToSet(data, ',');
        return true;
    }

    bool ReadFloatMap(const std::string &name, std::map<float, float> &val)
    {
        std::string data = GetConfigOption("Settings", name.c_str());
        if (data.empty()) {
            _WARNING("Failed to read FloatMap config option: %s", name.c_str());
            return false;
        }

        std::vector<std::string> pairs = SplitString(data, ',');
        for (auto &pair : pairs) {
            std::vector<std::string> kv = SplitString(pair, ':');
            if (kv.size() != 2) {
                _WARNING("Failed to read FloatMap entry %s for config option: %s", pair.c_str(), name.c_str());
                return false;
            }

            float k = std::stof(kv[0]);
            float v = std::stof(kv[1]);
            val[k] = v;
        }

        return true;
    }

    bool RegisterFloat(const std::string& name, float& val, bool isImportant=true)
    {
        if (!g_registrationComplete) floatMap[name] = &val;
        return ReadFloat(name, val, isImportant);
    }

    bool RegisterDouble(const std::string& name, double& val)
    {
        if (!g_registrationComplete) doubleMap[name] = &val;
        return ReadDouble(name, val);
    }

    bool RegisterInt(const std::string& name, int& val, bool isImportant=true)
    {
        if (!g_registrationComplete) intMap[name] = &val;
        return ReadInt(name, val, isImportant);
    }

    bool RegisterBool(const std::string& name, bool& val)
    {
        if (!g_registrationComplete) boolMap[name] = &val;
        return ReadBool(name, val);
    }

    bool SetSettingDouble(const std::string_view& name, double val)
    {
        if (auto it = doubleMap.find(name); it != doubleMap.end()) {
            *it->second = val;
            return true;
        }
        if (auto it = floatMap.find(name); it != floatMap.end()) {
            *it->second = float(val);
            return true;
        }
        if (auto it = intMap.find(name); it != intMap.end()) {
            *it->second = int(val);
            return true;
        }
        if (auto it = boolMap.find(name); it != boolMap.end()) {
            *it->second = bool(val);
            return true;
        }
        return false;
    }

    bool GetSettingDouble(const std::string_view& name, double& out)
    {
        if (auto it = doubleMap.find(name); it != doubleMap.end()) {
            out = *it->second;
            return true;
        }
        if (auto it = floatMap.find(name); it != floatMap.end()) {
            out = double(*it->second);
            return true;
        }
        if (auto it = intMap.find(name); it != intMap.end()) {
            out = double(*it->second);
            return true;
        }
        if (auto it = boolMap.find(name); it != boolMap.end()) {
            out = double(*it->second);
            return true;
        }
        return false;
    }

    bool ReadConfigOptions()
    {
        bool success = true;

        if (!RegisterBool("debugDrawControllers", options.debugDrawControllers)) success = false;

        if (!RegisterBool("spoofProjectileWeaponHitsAsIfBlocked", options.spoofProjectileWeaponHitsAsIfBlocked)) success = false;

        if (!ReadVector("PalmVector", options.palmVector)) success = false;
        if (!ReadVector("PointingVector", options.pointingVector)) success = false;
        if (!ReadVector("PalmPosition", options.palmPosition)) success = false;

        if (!ReadVector("HandCollisionBoxHalfExtents", options.handCollisionBoxHalfExtents)) success = false;
        if (!ReadVector("HandCollisionBoxOffset", options.handCollisionBoxOffset)) success = false;
        if (!RegisterFloat("HandCollisionBoxRadius", options.handCollisionBoxRadius)) success = false;

        if (!ReadVector("HandCollisionBoxHalfExtentsBeast", options.handCollisionBoxHalfExtentsBeast)) success = false;
        if (!ReadVector("HandCollisionBoxOffsetBeast", options.handCollisionBoxOffsetBeast)) success = false;
        if (!RegisterFloat("HandCollisionBoxRadiusBeast", options.handCollisionBoxRadiusBeast)) success = false;

        if (!ReadVector("RightShoulderHmdOffset", options.rightShoulderHmdOffset)) success = false;
        if (!RegisterFloat("RightShoulderRadius", options.rightShoulderRadius)) success = false;

        if (!ReadVector("LeftShoulderHmdOffset", options.leftShoulderHmdOffset)) success = false;
        if (!RegisterFloat("LeftShoulderRadius", options.leftShoulderRadius)) success = false;

        if (!ReadVector("MouthHmdOffset", options.mouthHmdOffset)) success = false;
        if (!RegisterFloat("MouthRadius", options.mouthRadius)) success = false;

        if (!ReadVector("SelectionBeamStretch", options.selectionBeamStretch)) success = false;
        
        if (!ReadVector("RolloverOffsetRight", options.rolloverOffsetRight)) success = false;
        if (!ReadVector("RolloverOffsetLeft", options.rolloverOffsetLeft)) success = false;
        if (!ReadVector("RolloverRotation", options.rolloverRotation)) success = false;

        //if (!ReadVector("rightToLeftPalmRotation", options.rightToLeftPalmRotation)) success = false;

        if (!RegisterFloat("FarCastRadius", options.farCastRadius)) success = false;
        if (!RegisterFloat("FarCastDistance", options.farCastDistance)) success = false;

        if (!RegisterFloat("NearCastRadius", options.nearCastRadius)) success = false;
        if (!RegisterFloat("NearCastDistance", options.nearCastDistance)) success = false;

        if (!RegisterFloat("WidePullGrabRadius", options.widePullGrabRadius)) success = false;

        if (!RegisterFloat("NearbyGrabBodyRadius", options.nearbyGrabBodyRadius)) success = false;

        float castDirectionRequiredHalfAngle;
        if (!RegisterFloat("CastDirectionRequiredHalfAngle", castDirectionRequiredHalfAngle)) success = false;
        options.requiredCastDotProduct = cosf(castDirectionRequiredHalfAngle * 0.0174533); // degrees to radians

        if (!RegisterDouble("SelectedFadeTime", options.selectedLeewayTime)) success = false;
        if (!RegisterDouble("TriggerPreemptTime", options.triggerPressedLeewayTime)) success = false;
        if (!RegisterDouble("InputLeewayTime", options.inputLeewayTime)) success = false;
        if (!RegisterDouble("ForceInputTime", options.forceInputTime)) success = false;
        if (!RegisterDouble("PullApplyVelocityTime", options.pullApplyVelocityTime)) success = false;
        if (!RegisterDouble("PullTrackHandTime", options.pullTrackHandTime)) success = false;
        if (!RegisterDouble("LootSpawnInTime", options.lootSpawnInTime)) success = false;
        if (!RegisterDouble("GrabFreezeNearbyVelocityTime", options.grabFreezeNearbyVelocityTime)) success = false;
        if (!RegisterDouble("PullHapticFadeTime", options.pullHapticFadeTime)) success = false;
        if (!RegisterDouble("GrabHapticFadeTime", options.grabHapticFadeTime)) success = false;
        if (!RegisterDouble("GrabStartMaxTime", options.grabStartMaxTime)) success = false;
        if (!RegisterDouble("ShoulderDropHapticFadeTime", options.shoulderDropHapticFadeTime)) success = false;
        if (!RegisterDouble("MouthDropHapticFadeTime", options.mouthDropHapticFadeTime)) success = false;
        if (!RegisterDouble("RolloverHideTime", options.rolloverHideTime)) success = false;
        if (!RegisterDouble("physicsGrabIgnoreHandDistanceTime", options.physicsGrabIgnoreHandDistanceTime)) success = false;

        if (!RegisterDouble("FingerAnimateEndTime", options.fingerAnimateEndTime)) success = false;
        if (!RegisterDouble("FingerAnimateEndDoubleSpeedTime", options.fingerAnimateEndDoubleSpeedTime)) success = false;
        if (!RegisterDouble("AfterDropFingerAnimateTime", options.afterDropFingerAnimateTime)) success = false;
        if (!RegisterDouble("FingerAnimateStartDoubleSpeedTime", options.fingerAnimateStartDoubleSpeedTime)) success = false;
        if (!RegisterDouble("FingerAnimateGrabDoubleSpeedTime", options.fingerAnimateGrabDoubleSpeedTime)) success = false;
        if (!RegisterDouble("WeaponCollisionDisableOnHitTime", options.weaponCollisionDisableOnHitTime)) success = false;
        if (!RegisterDouble("WeaponCollisionDisableOnHitDelay", options.weaponCollisionDisableOnHitDelay)) success = false;
        if (!RegisterDouble("TriggerGripIconSwitchTime", options.triggerGripIconSwitchTime)) success = false;

        if (!RegisterDouble("RolloverAfterGrabAlphaFadeInTime", options.rolloverAfterGrabAlphaFadeInTime)) success = false;
        if (!RegisterDouble("RolloverAfterDropAlphaFadeInTime", options.rolloverAfterDropAlphaFadeInTime)) success = false;

        if (!ReadInt("LogLevel", options.logLevel)) success = false;

        if (!RegisterFloat("GrabStartSpeed", options.grabStartSpeed)) success = false;
        if (!RegisterFloat("GrabStartAngularSpeed", options.grabStartAngularSpeed)) success = false;

        if (!RegisterFloat("PullSpeedThreshold", options.pullSpeedThreshold)) success = false;
        if (!RegisterFloat("lootSpeedThreshold", options.lootSpeedThreshold)) success = false;
        if (!RegisterFloat("lootToGrabSpeedThreshold", options.lootToGrabSpeedThreshold)) success = false;
        if (!RegisterDouble("lootToGrabLeewayTime", options.lootToGrabLeewayTime)) success = false;

        if (!RegisterFloat("RolloverScale", options.rolloverScale)) success = false;

        if (!RegisterFloat("ThrowVelocityThreshold", options.throwVelocityThreshold)) success = false;
        if (!RegisterFloat("ThrowVelocityBoostFactor", options.throwVelocityBoostFactor)) success = false;
        if (!RegisterDouble("throwIgnoreHandCollisionTime", options.throwIgnoreHandCollisionTime)) success = false;

        if (!RegisterFloat("ShoulderVelocityThreshold", options.shoulderVelocityThreshold)) success = false;
        if (!RegisterFloat("MouthVelocityThreshold", options.mouthVelocityThreshold)) success = false;

        if (!RegisterFloat("PullDestinationZOffset", options.pullDestinationZOffset)) success = false;
        if (!RegisterFloat("PulledAngularDamping", options.pulledAngularDamping)) success = false;
        if (!RegisterFloat("PulledGrabHandAdjustDistance", options.pulledGrabHandAdjustDistance)) success = false;
        if (!RegisterFloat("AngularVelocityMultiplier", options.angularVelocityMultiplier)) success = false;
        if (!RegisterFloat("TangentialVelocityLimit", options.tangentialVelocityLimit)) success = false;
        if (!RegisterFloat("TwoHandedRotationSnapSpeed", options.twoHandedRotationSnapSpeed)) success = false;

        if (!RegisterBool("enableWeaponTwoHanding", options.enableWeaponTwoHanding)) success = false;
        if (!RegisterBool("offhandAffectsTwoHandedRotation", options.offhandAffectsTwoHandedRotation)) success = false;
        if (!RegisterFloat("twoHandedHandToHandAlignmentFactor", options.twoHandedHandToHandAlignmentFactor)) success = false;
        if (!RegisterFloat("twoHandedHandToHandShiftFactor", options.twoHandedHandToHandShiftFactor)) success = false;
        if (!RegisterFloat("twoHandedHandToHandRotationFactor", options.twoHandedHandToHandRotationFactor)) success = false;

        if (!RegisterBool("offhandAffectsTwoHandedRotationCrossbow", options.offhandAffectsTwoHandedRotationCrossbow)) success = false;
        if (!RegisterFloat("twoHandedHandToHandAlignmentFactorCrossbow", options.twoHandedHandToHandAlignmentFactorCrossbow)) success = false;
        if (!RegisterFloat("twoHandedHandToHandShiftFactorCrossbow", options.twoHandedHandToHandShiftFactorCrossbow)) success = false;
        if (!RegisterFloat("twoHandedHandToHandRotationFactorCrossbow", options.twoHandedHandToHandRotationFactorCrossbow)) success = false;

        if (!RegisterFloat("SelectedCloseFingerAnimMaxHandSpeed", options.selectedCloseFingerAnimMaxHandSpeed)) success = false;
        if (!RegisterFloat("SelectedCloseFingerAnimValue", options.selectedCloseFingerAnimValue)) success = false;
        if (!RegisterFloat("FingerAnimateGrabLinearSpeed", options.fingerAnimateGrabLinearSpeed)) success = false;
        if (!RegisterFloat("FingerAnimateGrabAngularSpeed", options.fingerAnimateGrabAngularSpeed)) success = false;
        if (!RegisterFloat("FingerAnimateStartLinearSpeed", options.fingerAnimateStartLinearSpeed)) success = false;
        if (!RegisterFloat("FingerAnimateStartAngularSpeed", options.fingerAnimateStartAngularSpeed)) success = false;
        if (!RegisterFloat("FingerAnimateEndLinearSpeed", options.fingerAnimateEndLinearSpeed)) success = false;
        if (!RegisterFloat("FingerAnimateEndAngularSpeed", options.fingerAnimateEndAngularSpeed)) success = false;

        if (!RegisterFloat("SelectionLockedStartHapticStrength", options.selectionLockedStartHapticStrength)) success = false;
        if (!RegisterFloat("SelectionLockedStartHapticDuration", options.selectionLockedStartHapticDuration)) success = false;
        if (!RegisterFloat("SelectionLockedEndHapticStrength", options.selectionLockedEndHapticStrength)) success = false;
        if (!RegisterFloat("SelectionLockedEndHapticDuration", options.selectionLockedEndHapticDuration)) success = false;
        if (!RegisterFloat("SelectionLockedBaseHapticStrength", options.selectionLockedBaseHapticStrength)) success = false;
        if (!RegisterFloat("SelectionLockedProportionalHapticStrength", options.selectionLockedProportionalHapticStrength)) success = false;

        if (!RegisterFloat("GrabBaseHapticStrength", options.grabBaseHapticStrength)) success = false;
        if (!RegisterFloat("GrabProportionalHapticStrength", options.grabProportionalHapticStrength)) success = false;
        if (!RegisterFloat("GrabHapticMassExponent", options.grabHapticMassExponent)) success = false;

        if (!ReadInt("CollisionMaxInactiveFramesToConsiderActive", options.collisionMaxInactiveFramesToConsiderActive)) success = false;
        if (!ReadInt("CollisionMaxInactiveFramesBeforeCleanup", options.collisionMaxInactiveFramesBeforeCleanup)) success = false;
        if (!RegisterFloat("CollisionMaxInitialContactPointDistance", options.collisionMaxInitialContactPointDistance)) success = false;
        if (!RegisterFloat("CollisionMinHapticSpeed", options.collisionMinHapticSpeed)) success = false;
        if (!RegisterFloat("CollisionBaseHapticStrength", options.collisionBaseHapticStrength)) success = false;
        if (!RegisterFloat("CollisionMassProportionalHapticStrength", options.collisionMassProportionalHapticStrength)) success = false;
        if (!RegisterFloat("CollisionSpeedProportionalHapticStrength", options.collisionSpeedProportionalHapticStrength)) success = false;
        if (!RegisterFloat("CollisionHapticMassExponent", options.collisionHapticMassExponent)) success = false;
        if (!RegisterFloat("CollisionHapticDuration", options.collisionHapticDuration)) success = false;

        if (!RegisterFloat("ShoulderConstantHapticStrength", options.shoulderConstantHapticStrength)) success = false;
        if (!RegisterFloat("ShoulderDropHapticStrength", options.shoulderDropHapticStrength)) success = false;

        if (!RegisterFloat("MouthConstantHapticStrength", options.mouthConstantHapticStrength)) success = false;
        if (!RegisterFloat("MouthDropHapticStrength", options.mouthDropHapticStrength)) success = false;

        if (!RegisterFloat("NearbyGrabLinearDamping", options.nearbyGrabLinearDamping)) success = false;
        if (!RegisterFloat("NearbyGrabAngularDamping", options.nearbyGrabAngularDamping)) success = false;

        if (!RegisterFloat("NearbyGrabMaxLinearVelocity", options.nearbyGrabMaxLinearVelocity)) success = false;
        if (!RegisterFloat("NearbyGrabMaxAngularVelocity", options.nearbyGrabMaxAngularVelocity)) success = false;

        if (!RegisterFloat("PullDurationA", options.pullDurationA)) success = false;
        if (!RegisterFloat("PullDurationB", options.pullDurationB)) success = false;
        if (!RegisterFloat("PullDurationC", options.pullDurationC)) success = false;

        if (!RegisterFloat("MaxHandDistance", options.maxHandDistance)) success = false;
        if (!RegisterFloat("DampedCollisionHapticStrengthMultiplier", options.dampedCollisionHapticStrengthMultiplier)) success = false;

        if (!RegisterFloat("RolloverMinAlphaToShow", options.rolloverMinAlphaToShow)) success = false;
        if (!RegisterFloat("RolloverAlphaLogisticK", options.rolloverAlphaLogisticK)) success = false;
        if (!RegisterFloat("RolloverAlphaLogisticMidpoint", options.rolloverAlphaLogisticMidpoint)) success = false;
        if (!RegisterFloat("RolloverAlphaFadeInLogisticK", options.rolloverAlphaFadeInLogisticK)) success = false;
        if (!RegisterFloat("RolloverAlphaFadeInLogisticMidpoint", options.rolloverAlphaFadeInLogisticMidpoint)) success = false;

        if (!RegisterFloat("GeometryVertexAlphaThreshold", options.geometryVertexAlphaThreshold)) success = false;

        if (!RegisterFloat("GrabLateralWeight", options.grabLateralWeight)) success = false;
        if (!RegisterFloat("GrabDirectionalWeight", options.grabDirectionalWeight)) success = false;
        if (!RegisterFloat("grabMaxTriangleDistance", options.grabMaxTriangleDistance)) success = false;

        if (!RegisterBool("UseLoudSoundGrab", options.useLoudSoundGrab)) success = false;
        if (!RegisterBool("UseLoudSoundDrop", options.useLoudSoundDrop)) success = false;
        if (!RegisterBool("UseLoudSoundPull", options.useLoudSoundPull)) success = false;

        if (!RegisterBool("EnableWeaponCollision", options.enableWeaponCollision)) success = false;
        if (!RegisterBool("ForcePhysicsGrab", options.forcePhysicsGrab)) success = false;
        if (!RegisterBool("DisableGrabHairGeometry", options.disableGrabHair)) success = false;
        if (!RegisterBool("DisableGrabGeometryWithVertexAlpha", options.disableGrabGeometryWithVertexAlpha)) success = false;
        if (!RegisterBool("InheritTangentialVelocity", options.inheritTangentialVelocity)) success = false;

        if (!RegisterBool("useVrikWeaponTransform", options.useVrikWeaponTransform)) success = false;
        if (!RegisterFloat("weaponCollisionScale", options.weaponCollisionScale)) success = false;

        if (!RegisterBool("slowMovementWhenObjectIsHeld", options.slowMovementWhenObjectIsHeld)) success = false;
        if (!RegisterFloat("slowMovementMassProportion", options.slowMovementMassProportion)) success = false;
        if (!RegisterFloat("slowMovementMassExponent", options.slowMovementMassExponent)) success = false;
        if (!RegisterFloat("slowMovementMaxReduction", options.slowMovementMaxReduction)) success = false;
        if (!RegisterDouble("slowMovementFadeOutTime", options.slowMovementFadeOutTime)) success = false;

        if (!RegisterFloat("jumpHeightMassProportion", options.jumpHeightMassProportion)) success = false;
        if (!RegisterFloat("jumpHeightMassExponent", options.jumpHeightMassExponent)) success = false;
        if (!RegisterFloat("jumpHeightMaxReduction", options.jumpHeightMaxReduction)) success = false;

        if (!RegisterBool("EnableHavokFix", options.enableHavokFix)) success = false;
        if (!RegisterFloat("HavokMaxTimeComplexMultiplier", options.havokMaxTimeComplexMultiplier)) success = false;
        if (!RegisterFloat("minPhysicsFrameRate", options.minPhysicsFrameRate)) success = false;
        if (!RegisterInt("maxNumPhysicsStepsPerUpdate", options.maxNumPhysicsStepsPerUpdate)) success = false;
        if (!RegisterInt("maxNumPhysicsStepsPerUpdateComplex", options.maxNumPhysicsStepsPerUpdateComplex)) success = false;

        if (!RegisterBool("EnableShadowUpdateFix", options.enableShadowUpdateFix)) success = false;
        if (!RegisterInt("numShadowUpdates", options.numShadowUpdates)) success = false;
        if (!ReadInt("MaxNumEntitiesPerSimulationIslandToCheck", options.maxNumEntitiesPerSimulationIslandToCheck)) success = false;
        if (!RegisterFloat("MaxDistanceOfSimulationIslandToUpdate", options.maxDistanceOfSimulationIslandToUpdate)) success = false;

        if (!RegisterBool("forceGrabbedNodeUpdate", options.forceGrabbedNodeUpdate)) success = false;

        if (!RegisterFloat("grabConstraintAngularTau", options.grabConstraintAngularTau)) success = false;
        if (!RegisterFloat("grabConstraintAngularProportionalRecoveryVelocity", options.grabConstraintAngularProportionalRecoveryVelocity)) success = false;
        if (!RegisterFloat("grabConstraintAngularConstantRecoveryVelocity", options.grabConstraintAngularConstantRecoveryVelocity)) success = false;
        if (!RegisterFloat("grabConstraintAngularDamping", options.grabConstraintAngularDamping)) success = false;

        if (!RegisterFloat("grabConstraintLinearTau", options.grabConstraintLinearTau)) success = false;
        if (!RegisterFloat("grabConstraintLinearMaxForce", options.grabConstraintLinearMaxForce)) success = false;
        if (!RegisterFloat("grabConstraintLinearMaxForceWeapon", options.grabConstraintLinearMaxForceWeapon)) success = false;
        if (!RegisterFloat("grabConstraintLinearProportionalRecoveryVelocity", options.grabConstraintLinearProportionalRecoveryVelocity)) success = false;
        if (!RegisterFloat("grabConstraintLinearConstantRecoveryVelocity", options.grabConstraintLinearConstantRecoveryVelocity)) success = false;
        if (!RegisterFloat("grabConstraintLinearDamping", options.grabConstraintLinearDamping)) success = false;

        if (!RegisterFloat("grabConstraintAngularTauBody", options.grabConstraintAngularTauBody)) success = false;
        if (!RegisterFloat("grabConstraintAngularTauBodyStart", options.grabConstraintAngularTauBodyStart)) success = false;
        if (!RegisterFloat("grabConstraintLinearTauBody", options.grabConstraintLinearTauBody)) success = false;
        if (!RegisterFloat("grabConstraintLinearTauBodyStart", options.grabConstraintLinearTauBodyStart)) success = false;

        if (!RegisterDouble("physicsGrabLerpTauTimeBody", options.physicsGrabLerpTauTimeBody)) success = false;

        if (!RegisterFloat("grabConstraintAngularTauActor", options.grabConstraintAngularTauActor)) success = false;
        if (!RegisterFloat("grabConstraintLinearTauActor", options.grabConstraintLinearTauActor)) success = false;

        if (!RegisterFloat("grabConstraintCollidingAngularTau", options.grabConstraintCollidingAngularTau)) success = false;
        if (!RegisterFloat("grabConstraintCollidingLinearTau", options.grabConstraintCollidingLinearTau)) success = false;
        if (!RegisterFloat("grabConstraintTauLerpSpeed", options.grabConstraintTauLerpSpeed)) success = false;

        if (!RegisterFloat("grabConstraintMaxForceToMassRatio", options.grabConstraintMaxForceToMassRatio)) success = false;
        if (!RegisterFloat("grabConstraintAngularToLinearForceRatio", options.grabConstraintAngularToLinearForceRatio)) success = false;

        if (!RegisterFloat("grabConstraintFadeInStartAngularMaxForceRatio", options.grabConstraintFadeInStartAngularMaxForceRatio)) success = false;
        if (!RegisterDouble("grabConstraintFadeInTime", options.grabConstraintFadeInTime)) success = false;
        
        if (!RegisterFloat("grabConstraintLinearMaxForceActor", options.grabConstraintLinearMaxForceActor)) success = false;
        if (!RegisterFloat("grabConstraintAngularMaxForceActor", options.grabConstraintAngularMaxForceActor)) success = false;

        if (!ReadFloatMap("fpsToActorMaxForceMultiplierMapLinear", options.fpsToActorMaxForceMultiplierMapLinear)) success = false;
        if (!ReadFloatMap("fpsToActorMaxForceMultiplierMapAngular", options.fpsToActorMaxForceMultiplierMapAngular)) success = false;

        if (!RegisterDouble("physicsGrabLerpHandTimeMin", options.physicsGrabLerpHandTimeMin)) success = false;
        if (!RegisterDouble("physicsGrabLerpHandTimeMax", options.physicsGrabLerpHandTimeMax)) success = false;
        if (!RegisterFloat("physicsGrabLerpHandMinDistance", options.physicsGrabLerpHandMinDistance)) success = false;
        if (!RegisterFloat("physicsGrabLerpHandMaxDistance", options.physicsGrabLerpHandMaxDistance)) success = false;

        if (!RegisterDouble("sneakUnsneakIgnoreHandDistanceTime", options.sneakUnsneakIgnoreHandDistanceTime)) success = false;
        if (!RegisterDouble("handWeaponCollisionEnableDelay", options.handWeaponCollisionEnableDelay)) success = false;

        if (!RegisterFloat("minCollideClutterMass", options.minCollideClutterMass)) success = false;

        if (!RegisterBool("enableTwoHandedGrabbing", options.enableTwoHandedGrabbing)) success = false;
        if (!RegisterBool("allowGrabWithSpell", options.allowGrabWithSpell)) success = false;
        if (!RegisterBool("restrictPullWithSpell", options.restrictPullWithSpell)) success = false;

        if (!RegisterFloat("grabbedObjectMinInertia", options.grabbedObjectMinInertia)) success = false;
        if (!RegisterFloat("grabbedObjectMaxInertiaRatio", options.grabbedObjectMaxInertiaRatio)) success = false;

        if (!RegisterBool("doPhysicsGrabPlayerMovementCompensation", options.doPhysicsGrabPlayerMovementCompensation)) success = false;
        if (!RegisterFloat("playerSpaceMinDeltaAngleToWarp", options.playerSpaceMinDeltaAngleToWarp)) success = false;
        if (!RegisterInt("grabbedActorAffectedBoneRadius", options.grabbedActorAffectedBoneRadius)) success = false;

        if (!RegisterFloat("droppedObjMinDetectionSpeed", options.droppedObjMinDetectionSpeed)) success = false;
        if (!RegisterFloat("droppedObjDetectionMassSilent", options.droppedObjDetectionMassSilent)) success = false;
        if (!RegisterFloat("droppedObjDetectionMassNormal", options.droppedObjDetectionMassNormal)) success = false;
        if (!RegisterFloat("droppedObjDetectionMassLoud", options.droppedObjDetectionMassLoud)) success = false;

        if (!RegisterFloat("droppedObjMinDestructibleSpeed", options.droppedObjMinDestructibleSpeed)) success = false;
        if (!RegisterFloat("droppedObjDestructibleInflictedDamage", options.droppedObjDestructibleInflictedDamage)) success = false;
        if (!RegisterFloat("droppedObjDestructibleSelfDamage", options.droppedObjDestructibleSelfDamage)) success = false;

        if (!RegisterBool("DisableShaders", options.disableShaders)) success = false;
        if (!RegisterBool("DisableSelectionBeam", options.disableSelectionBeam)) success = false;
        if (!RegisterBool("DisableLooting", options.disableLooting)) success = false;
        if (!RegisterBool("disableGravityGlovesLooting", options.disableGravityGlovesLooting)) success = false;
        if (!RegisterBool("disableGravityGlovesLootingLiveActors", options.disableGravityGlovesLootingLiveActors)) success = false;
        if (!RegisterBool("allowLootingNonRagdolledActors", options.allowLootingNonRagdolledActors)) success = false;
        if (!RegisterBool("allowLootingLiveActors", options.allowLootingLiveActors)) success = false;
        if (!RegisterBool("SkipActivateBooks", options.skipActivateBooks)) success = false;
        if (!RegisterBool("DisableRolloverRumble", options.disableRolloverRumble)) success = false;
        if (!RegisterBool("AlwaysShowHands", options.alwaysShowHands)) success = false;
        if (!RegisterBool("DisableVanillaGrab", options.disableVanillaGrab)) success = false;
        if (!RegisterBool("convertDebrisToMoving", options.convertDebrisToMoving)) success = false;

        if (!RegisterBool("treatHandCollisionAsBelongingToPlayer", options.treatHandCollisionAsBelongingToPlayer)) success = false;
        if (!RegisterBool("allowAllPlayerCollisionForTriggers", options.allowAllPlayerCollisionForTriggers)) success = false;

        if (!RegisterBool("AllowGrabWithEmptyArrowHand", options.allowGrabWithEmptyArrowHand)) success = false;
        if (!RegisterBool("AllowGrabWithTwoHandedOffhand", options.allowGrabWithTwoHandedOffhand)) success = false;
        if (!RegisterBool("allowDaggerTwoHanding", options.allowDaggerTwoHanding)) success = false;
        if (!RegisterBool("AllowTwoHandingWithSpellInOffhand", options.allowTwoHandingWithSpellInOffhand)) success = false;
        if (!RegisterBool("AllowGrabCurrentHorse", options.allowGrabCurrentHorse)) success = false;

        if (!RegisterBool("grabIgnoreBlood", options.grabIgnoreBlood)) success = false;
        if (!RegisterBool("grabIgnoreDecal", options.grabIgnoreDecal)) success = false;
        if (!RegisterBool("grabIgnoreSoftEffect", options.grabIgnoreSoftEffect)) success = false;

        if (!RegisterBool("EnableTrigger", options.enableTrigger)) success = false;
        if (!RegisterBool("EnableGrip", options.enableGrip)) success = false;

        if (!RegisterBool("EnableDrinkPoison", options.enableDrinkPoison)) success = false;
        if (!RegisterBool("OverrideActivateText", options.overrideActivateText)) success = false;
        if (!RegisterBool("UseAttachPointForInitialGrab", options.useAttachPointForInitialGrab)) success = false;

        if (!RegisterBool("dontAnimateFingersWhenBeast", options.dontAnimateFingersWhenBeast)) success = false;

        if (!RegisterBool("doContainerPhysics", options.doContainerPhysics)) success = false;

        if (!RegisterBool("doDoublePrecision", options.doDoublePrecision)) success = false;
        if (!RegisterBool("handleVrikOffsetting", options.handleVrikOffsetting)) success = false;

        if (!RegisterBool("reloadConfigIfModified", options.reloadConfigIfModified)) success = false;

        if (!RegisterBool("enableHiggsGrabNodes", options.enableHiggsGrabNodes)) success = false;
        if (!RegisterBool("printHiggsGrabNodeInfo", options.printHiggsGrabNodeInfo)) success = false;

        if (!RegisterBool("DelayRightGripInput", options.delayRightGripInput)) success = false;
        if (!RegisterBool("DelayLeftGripInput", options.delayLeftGripInput)) success = false;

        if (!ReadString("GrabString", Config::options.grabString)) success = false;
        if (!ReadString("PullString", Config::options.pullString)) success = false;
        if (!ReadString("LootString", Config::options.lootString)) success = false;

        if (!ReadStringSet("GrabNodeNameBlacklist", Config::options.grabNodeNameBlacklist)) success = false;

        RegisterFloat("dummyFloat0", options.dummyFloat0, false);
        RegisterFloat("dummyFloat1", options.dummyFloat1, false);
        RegisterFloat("dummyFloat2", options.dummyFloat2, false);
        RegisterFloat("dummyFloat3", options.dummyFloat3, false);
        RegisterFloat("dummyFloat4", options.dummyFloat4, false);

        RegisterInt("dummyInt0", options.dummyInt0, false);
        RegisterInt("dummyInt1", options.dummyInt1, false);
        RegisterInt("dummyInt2", options.dummyInt2, false);
        RegisterInt("dummyInt3", options.dummyInt3, false);
        RegisterInt("dummyInt4", options.dummyInt4, false);

        g_registrationComplete = true;

        return success;
    }

    bool ReloadIfModified()
    {
        namespace fs = std::filesystem;

        static long long lastModifiedConfigTime = 0;

        const std::string &path = GetConfigPath();
        auto ftime = fs::last_write_time(path);
        auto time = ftime.time_since_epoch().count();
        if (time > lastModifiedConfigTime) {
            lastModifiedConfigTime = time;

            // Reload config if file has been modified since we last read it
            if (Config::ReadConfigOptions()) {
                _MESSAGE("Successfully reloaded config parameters");
            }
            else {
                _WARNING("[WARNING] Failed to reload config options");
            }

            return true;
        }

        return false;
    }

    const std::string & GetConfigPath()
    {
        static std::string s_configPath;

        if (s_configPath.empty()) {
            std::string	runtimePath = GetRuntimeDirectory();
            if (!runtimePath.empty()) {
                s_configPath = runtimePath + "Data\\SKSE\\Plugins\\higgs_vr.ini";

                _MESSAGE("config path = %s", s_configPath.c_str());
            }
        }

        return s_configPath;
    }

    std::string GetConfigOption(const char *section, const char *key)
    {
        std::string	result;

        const std::string & configPath = GetConfigPath();
        if (!configPath.empty()) {
            char	resultBuf[256];
            resultBuf[0] = 0;

            UInt32	resultLen = GetPrivateProfileString(section, key, NULL, resultBuf, sizeof(resultBuf), configPath.c_str());

            result = resultBuf;
        }

        return result;
    }

    bool GetConfigOptionDouble(const char *section, const char *key, double *out)
    {
        std::string	data = GetConfigOption(section, key);
        if (data.empty())
            return false;

        *out = std::stod(data);
        return true;
    }

    bool GetConfigOptionFloat(const char *section, const char *key, float *out)
    {
        std::string	data = GetConfigOption(section, key);
        if (data.empty())
            return false;

        *out = std::stof(data);
        return true;
    }

    bool GetConfigOptionInt(const char *section, const char *key, int *out)
    {
        std::string	data = GetConfigOption(section, key);
        if (data.empty())
            return false;

        *out = std::stoi(data);
        return true;
    }

    bool GetConfigOptionBool(const char *section, const char *key, bool *out)
    {
        std::string	data = GetConfigOption(section, key);
        if (data.empty())
            return false;

        int val = std::stoi(data);
        if (val == 1) {
            *out = true;
            return true;
        }
        else if (val == 0) {
            *out = false;
            return true;
        }
        else {
            return false;
        }
    }
}
