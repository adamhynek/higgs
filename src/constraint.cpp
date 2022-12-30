#pragma once

#include "RE/havok.h"
#include "RE/offsets.h"
#include "constraint.h"
#include "utils.h"
#include "config.h"


GrabConstraintData::GrabConstraintData() : hkpConstraintData() {
	m_referenceCount = 1;
	m_userData = 0;

	m_atoms.m_transforms.m_transformA.setIdentity();
	m_atoms.m_transforms.m_transformB.setIdentity();

	m_atoms.m_ragdollMotors.m_isEnabled = false;
	m_atoms.m_ragdollMotors.m_initializedOffset = offsetof(Runtime, m_initialized);
	m_atoms.m_ragdollMotors.m_previousTargetAnglesOffset = offsetof(Runtime, m_previousTargetAngles);
	m_atoms.m_ragdollMotors.m_target_bRca.setIdentity();
	m_atoms.m_ragdollMotors.m_motors[0] = nullptr;
	m_atoms.m_ragdollMotors.m_motors[1] = nullptr;
	m_atoms.m_ragdollMotors.m_motors[2] = nullptr;

	hkpPositionConstraintMotor *motor = hkAllocReferencedObject<hkpPositionConstraintMotor>();
	hkpPositionConstraintMotor_ctor(motor);

	motor->m_tau = Config::options.grabConstraintTau;
	motor->setMaxForce(Config::options.grabConstraintMaxForce);
	motor->m_proportionalRecoveryVelocity = Config::options.grabConstraintProportionalRecoveryVelocity;
	motor->m_constantRecoveryVelocity = Config::options.grabConstraintConstantRecoveryVelocity;
	motor->m_damping = Config::options.grabConstraintDamping;

	setMotor(0, motor);
	setMotor(1, motor);
	setMotor(2, motor);

	hkReferencedObject_removeReference(motor);
}

GrabConstraintData::~GrabConstraintData() {
	// decref motors
	setMotor(0, nullptr);
	setMotor(1, nullptr);
	setMotor(2, nullptr);
}

void GrabConstraintData::setInBodySpace(const hkTransform &transformA, const hkTransform &transformB)
{
	m_atoms.m_transforms.m_transformA = transformA;
	m_atoms.m_transforms.m_transformB = transformB;
}

void GrabConstraintData::setMaxLinearImpulse(hkReal maxImpulse)
{
	m_atoms.m_ballSocket.m_maxImpulse = maxImpulse;
}

hkReal GrabConstraintData::getMaxLinearImpulse() const
{
	return m_atoms.m_ballSocket.m_maxImpulse;
}

void GrabConstraintData::setBodyToNotify(int bodyIdx)
{
	m_atoms.m_ballSocket.m_bodiesToNotify = 1 << bodyIdx;
}

hkUint8 GrabConstraintData::getNotifiedBodyIndex() const
{
	return m_atoms.m_ballSocket.m_bodiesToNotify >> 1;
}

hkBool GrabConstraintData::isValid() const
{
	return m_atoms.m_ballSocket.m_solvingMethod != hkpConeLimitConstraintAtom::SolvingMethod::METHOD_STABILIZED || m_atoms.m_setupStabilization.m_enabled;
}

int GrabConstraintData::getType() const
{
	return hkpConstraintData::ConstraintType::CONSTRAINT_TYPE_CUSTOM;
}

void GrabConstraintData::setSolvingMethod(hkpConstraintAtom::SolvingMethod method)
{
	if (method == hkpConstraintAtom::SolvingMethod::METHOD_STABILIZED) {
		m_atoms.m_setupStabilization.m_enabled = true;
	}
	else {
		m_atoms.m_setupStabilization.m_enabled = false;
	}

	m_atoms.m_ballSocket.m_solvingMethod = method;
}

hkResult GrabConstraintData::getInertiaStabilizationFactor(hkReal &inertiaStabilizationFactorOut) const
{
	inertiaStabilizationFactorOut = m_atoms.m_ballSocket.getInertiaStabilizationFactor();
	return hkResult::HK_SUCCESS;
}

hkResult GrabConstraintData::setInertiaStabilizationFactor(const hkReal inertiaStabilizationFactorIn)
{
	m_atoms.m_ballSocket.setInertiaStabilizationFactor(std::clamp(inertiaStabilizationFactorIn, 0.f, 1.f));
	return hkResult::HK_SUCCESS;
}

void GrabConstraintData::getConstraintInfo(hkpConstraintData::ConstraintInfo &infoOut) const
{
	hkpConstraintData_getConstraintInfoUtil(m_atoms.getAtoms(), m_atoms.getSizeOfAllAtoms(), infoOut);
}

void GrabConstraintData::getRuntimeInfo(hkBool wantRuntime, hkpConstraintData::RuntimeInfo &infoOut) const
{
	if (wantRuntime || m_atoms.m_ballSocket.m_maxImpulse != HK_REAL_MAX) {
		infoOut.m_numSolverResults = SOLVER_RESULT_MAX;
		infoOut.m_sizeOfExternalRuntime = sizeof(Runtime);
	}
	else {
		infoOut.m_numSolverResults = 0;
		infoOut.m_sizeOfExternalRuntime = 0;
	}
}

void GrabConstraintData::addInstance(hkpConstraintInstance *constraint, hkpConstraintRuntime *runtime, int sizeOfRuntime) const
{
	if (runtime) {
		memset(runtime, 0, sizeOfRuntime);
	}
}

void GrabConstraintData::setMotor(int index, hkpConstraintMotor *newMotor)
{
	if (newMotor) {
		//hkReferencedObject_addReference(newMotor);
		hkReferencedObject_addReference((hkReferencedObject *)newMotor);
	}
	hkpConstraintMotor *&motor = m_atoms.m_ragdollMotors.m_motors[index];
	if (motor) {
		//hkReferencedObject_removeReference(motor);
		hkReferencedObject_removeReference((hkReferencedObject *)motor);
	}
	motor = newMotor;
}

void GrabConstraintData::setMotorsActive(hkpConstraintInstance *instance, hkBool toBeEnabled)
{
	m_atoms.m_ragdollMotors.m_isEnabled = toBeEnabled;
	if (!toBeEnabled) {
		if (hkConstraintInternal *constraintInternal = instance->getInternal()) {
			if (Runtime *runtime = getRuntime(constraintInternal->m_runtime)) {
				runtime->m_solverResults[SOLVER_RESULT_MOTOR_0].init();
				runtime->m_solverResults[SOLVER_RESULT_MOTOR_1].init();
				runtime->m_solverResults[SOLVER_RESULT_MOTOR_2].init();
			}
		}
	}
}

void GrabConstraintData::setTarget(const hkMatrix3 &target_cbRca)
{
	hkMatrix3_setMul(m_atoms.m_ragdollMotors.m_target_bRca, m_atoms.m_transforms.m_transformB.m_rotation, target_cbRca);
}

void GrabConstraintData::setTargetRelativeOrientationOfBodies(const hkRotation &bRa)
{
	hkMatrix3_setMul(m_atoms.m_ragdollMotors.m_target_bRca, bRa, m_atoms.m_transforms.m_transformA.m_rotation);
}
