#pragma once

#include "RE/havok.h"


class GrabConstraintData : public hkpConstraintData
{
public:
	inline void *operator new(hk_size_t nbytes) {
		hkReferencedObject *b = static_cast<hkReferencedObject *>(hkHeapAlloc(static_cast<int>(nbytes)));
		b->m_memSizeAndFlags = static_cast<hkUint16>(nbytes);
		return b;
	}

	inline void operator delete(void *p) {
		hkReferencedObject *b = static_cast<hkReferencedObject *>(p);
		hkGetMemoryRouter().heap().blockFree(p, b->m_memSizeAndFlags);
	}

	inline void *operator new(hk_size_t, void *p) { return p; }
	inline void *operator new[](hk_size_t, void *p) { HK_BREAKPOINT(0); return p; }
	inline void operator delete(void *, void *) { }
	inline void operator delete[](void *, void *) { HK_BREAKPOINT(0); }

	GrabConstraintData();

	virtual ~GrabConstraintData();

	void setInBodySpace(const hkTransform &transformA, const hkTransform &transformB);

	virtual const hkClass *getClassType() const { return 0; }

	virtual void calcContentStatistics(hkStatisticsCollector *collector, const hkClass *cls) const { return; }

	virtual void setMaxLinearImpulse(hkReal maxImpulse);

	/// Gets the maximum impulse that can be applied by this constraint.
	virtual hkReal getMaxLinearImpulse() const;

	/// Choose the body to be notified when the constraint's maximum impulse is breached.
	virtual void setBodyToNotify(int bodyIdx);

	/// Returns the index of the body that is notified when the constraint's maximum impulse is breached.
	virtual hkUint8 getNotifiedBodyIndex() const;

	/// Check consistency of constraint members.
	virtual hkBool isValid() const;

	/// Get type from this constraint.
	virtual int getType() const;

	/// Sets the solving method for this constraint. Use one of the hkpConstraintAtom::SolvingMethod as a value for method.
	virtual void setSolvingMethod(hkpConstraintAtom::SolvingMethod method);

	/// Gets the inertia stabilization factor, returns HK_FAILURE if the factor is not defined for the given constraint.
	virtual hkResult getInertiaStabilizationFactor(hkReal &inertiaStabilizationFactorOut) const;

	/// Sets the inertia stabilization factor, return HK_FAILURE if the factor is not defined for the given constraint.
	virtual hkResult setInertiaStabilizationFactor(const hkReal inertiaStabilizationFactorIn);

	//
	//	Solver interface
	//
	enum
	{
		SOLVER_RESULT_MOTOR_0 = 0,		// the angular motor value
		SOLVER_RESULT_MOTOR_1 = 1,		// the angular motor value
		SOLVER_RESULT_MOTOR_2 = 2,		// the angular motor value

		SOLVER_RESULT_MOTOR_3 = 3,		// the linear motor value
		SOLVER_RESULT_MOTOR_4 = 4,		// the linear motor value
		SOLVER_RESULT_MOTOR_5 = 5,		// the linear motor value

		SOLVER_RESULT_MAX = 6
	};

	struct Runtime
	{
		class hkpSolverResults m_solverResults[SOLVER_RESULT_MAX]; // 00

		// for angular constraint motors
		hkUint8 m_initialized[3]; // 30
		hkReal m_previousTargetAngles[3]; // 34

		// for linear constraint motors
		hkUint8 m_initializedLinear[3];
		hkReal m_previousTargetPositions[3];
	};

	inline Runtime *getRuntime(hkpConstraintRuntime *runtime) { return reinterpret_cast<Runtime *>(runtime); }

	struct Atoms
	{
		struct hkpSetLocalTransformsConstraintAtom		m_transforms;
		struct hkpSetupStabilizationAtom				m_setupStabilization;
		struct hkpRagdollMotorConstraintAtom			m_ragdollMotors;
		struct hkpLinMotorConstraintAtom				m_linearMotor0;
		struct hkpLinMotorConstraintAtom				m_linearMotor1;
		struct hkpLinMotorConstraintAtom				m_linearMotor2;

		Atoms() {	}

		// get a pointer to the first atom
		const hkpConstraintAtom *getAtoms() const { return &m_transforms; }

		int getSizeOfAllAtoms() const { return hkGetByteOffsetInt(this, &m_linearMotor2 + 1); }
	};

	__declspec(align(16)) struct Atoms m_atoms; // 20

	//
	// Internal functions.
	//
	// hkpConstraintData interface implementations
	virtual void getConstraintInfo(hkpConstraintData::ConstraintInfo &infoOut) const;

	// hkpConstraintData interface implementations
	virtual void getRuntimeInfo(hkBool wantRuntime, hkpConstraintData::RuntimeInfo &infoOut) const;

	/// Access to the solver results in a generic way. Use getRuntimeInfo to get the number of solver results.
	virtual hkpSolverResults *getSolverResults(hkpConstraintRuntime *runtime) { return getRuntime(runtime)->m_solverResults; } // 0D

	/// Initialize the runtime data. The default implementation simply zeros all values.
	virtual void addInstance(hkpConstraintInstance *constraint, hkpConstraintRuntime *runtime, int sizeOfRuntime) const;

	void setMotor(int index, hkpConstraintMotor *newMotor);

	void setMotorsActive(hkpConstraintInstance *instance, hkBool toBeEnabled);

	void setTarget(const hkMatrix3 &target_cbRca);

	void setTargetRelativeOrientationOfBodies(const hkRotation &bRa);
};
