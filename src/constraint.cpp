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

	motor->m_tau = Config::options.grabConstraintAngularTau;
	motor->setMaxForce(Config::options.grabConstraintAngularMaxForce);
	motor->m_proportionalRecoveryVelocity = Config::options.grabConstraintAngularProportionalRecoveryVelocity;
	motor->m_constantRecoveryVelocity = Config::options.grabConstraintAngularConstantRecoveryVelocity;
	motor->m_damping = Config::options.grabConstraintAngularDamping;

	setMotor(0, motor);
	setMotor(1, motor);
	setMotor(2, motor);

	hkReferencedObject_removeReference(motor);


	m_atoms.m_linearMotor0.m_isEnabled = false;
	m_atoms.m_linearMotor0.m_motorAxis = 0;
	m_atoms.m_linearMotor0.m_initializedOffset = offsetof(Runtime, m_initializedLinear[0]);
	m_atoms.m_linearMotor0.m_previousTargetPositionOffset = offsetof(Runtime, m_previousTargetPositions[0]);
	m_atoms.m_linearMotor0.m_targetPosition = 0.f;
	m_atoms.m_linearMotor0.m_motor = nullptr;

	m_atoms.m_linearMotor1.m_isEnabled = false;
	m_atoms.m_linearMotor1.m_motorAxis = 1;
	m_atoms.m_linearMotor1.m_initializedOffset = offsetof(Runtime, m_initializedLinear[1]);
	m_atoms.m_linearMotor1.m_previousTargetPositionOffset = offsetof(Runtime, m_previousTargetPositions[1]);
	m_atoms.m_linearMotor1.m_targetPosition = 0.f;
	m_atoms.m_linearMotor1.m_motor = nullptr;

	m_atoms.m_linearMotor2.m_isEnabled = false;
	m_atoms.m_linearMotor2.m_motorAxis = 2;
	m_atoms.m_linearMotor2.m_initializedOffset = offsetof(Runtime, m_initializedLinear[2]);
	m_atoms.m_linearMotor2.m_previousTargetPositionOffset = offsetof(Runtime, m_previousTargetPositions[2]);
	m_atoms.m_linearMotor2.m_targetPosition = 0.f;
	m_atoms.m_linearMotor2.m_motor = nullptr;

	hkpPositionConstraintMotor *linearMotor = hkAllocReferencedObject<hkpPositionConstraintMotor>();
	hkpPositionConstraintMotor_ctor(linearMotor);

	linearMotor->m_tau = Config::options.grabConstraintLinearTau;
	linearMotor->setMaxForce(Config::options.grabConstraintLinearMaxForce);
	linearMotor->m_proportionalRecoveryVelocity = Config::options.grabConstraintLinearProportionalRecoveryVelocity;
	linearMotor->m_constantRecoveryVelocity = Config::options.grabConstraintLinearConstantRecoveryVelocity;
	linearMotor->m_damping = Config::options.grabConstraintLinearDamping;

	setMotor(3, linearMotor);
	setMotor(4, linearMotor);
	setMotor(5, linearMotor);

	hkReferencedObject_removeReference(linearMotor);
}

GrabConstraintData::~GrabConstraintData() {
	// decref motors
	setMotor(0, nullptr);
	setMotor(1, nullptr);
	setMotor(2, nullptr);

	setMotor(3, nullptr);
	setMotor(4, nullptr);
	setMotor(5, nullptr);
}

void GrabConstraintData::setInBodySpace(const hkTransform &transformA, const hkTransform &transformB)
{
	m_atoms.m_transforms.m_transformA = transformA;
	m_atoms.m_transforms.m_transformB = transformB;
}

hkBool GrabConstraintData::isValid() const
{
	return true;
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
}

void GrabConstraintData::getConstraintInfo(hkpConstraintData::ConstraintInfo &infoOut) const
{
	hkpConstraintData_getConstraintInfoUtil(m_atoms.getAtoms(), m_atoms.getSizeOfAllAtoms(), infoOut);
}

void GrabConstraintData::getRuntimeInfo(hkBool wantRuntime, hkpConstraintData::RuntimeInfo &infoOut) const
{
	if (wantRuntime) {
		infoOut.m_numSolverResults = SOLVER_RESULT_MAX;
		infoOut.m_sizeOfExternalRuntime = Runtime::getSizeOfExternalRuntime();
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
	hkpConstraintMotor *&motor = index < 3 ? m_atoms.m_ragdollMotors.m_motors[index] : (index == 3 ? m_atoms.m_linearMotor0.m_motor : (index == 4 ? m_atoms.m_linearMotor1.m_motor : m_atoms.m_linearMotor2.m_motor));
	if (motor) {
		//hkReferencedObject_removeReference(motor);
		hkReferencedObject_removeReference((hkReferencedObject *)motor);
	}
	motor = newMotor;
}

void GrabConstraintData::setMotorsActive(hkpConstraintInstance *instance, hkBool toBeEnabled)
{
	m_atoms.m_ragdollMotors.m_isEnabled = toBeEnabled;
	m_atoms.m_linearMotor0.m_isEnabled = toBeEnabled;
	m_atoms.m_linearMotor1.m_isEnabled = toBeEnabled;
	m_atoms.m_linearMotor2.m_isEnabled = toBeEnabled;
	if (!toBeEnabled) {
		if (hkConstraintInternal *constraintInternal = instance->getInternal()) {
			if (Runtime *runtime = getRuntime(constraintInternal->m_runtime)) {
				runtime->m_solverResults[SOLVER_RESULT_MOTOR_0].init();
				runtime->m_solverResults[SOLVER_RESULT_MOTOR_1].init();
				runtime->m_solverResults[SOLVER_RESULT_MOTOR_2].init();

				runtime->m_solverResults[SOLVER_RESULT_MOTOR_3].init();
				runtime->m_solverResults[SOLVER_RESULT_MOTOR_4].init();
				runtime->m_solverResults[SOLVER_RESULT_MOTOR_5].init();
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
