#pragma once

#include "RE/havok.h"
#include "RE/offsets.h"
#include "utils.h"

hkMemoryRouter &hkGetMemoryRouter()
{
	return *(hkMemoryRouter *)(hkUlong)TlsGetValue(*g_havokMemoryRouterTlsIndex);
}

hkUFloat8 &hkUFloat8::operator=(const float &fv)
{
	hkRealTohkUFloat8(*this, fv);
	return *this;
}

auto hkConstraintCinfo_vtbl = RelocAddr<void *>(0x161D218);
hkConstraintCinfo::hkConstraintCinfo()
{
	set_vtbl(this, hkConstraintCinfo_vtbl);
}

hkConstraintCinfo::~hkConstraintCinfo()
{
	hkConstraintCinfo_setConstraintData(this, nullptr);
}

auto hkBallAndSocketConstraintCinfo_vtbl = RelocAddr<void *>(0x18249C8);
hkBallAndSocketConstraintCinfo::hkBallAndSocketConstraintCinfo(hkpRigidBody *rigidBodyA, hkpRigidBody *rigidBodyB, NiPoint3 &pivotA, NiPoint3 &pivotB)
{
	set_vtbl(this, hkBallAndSocketConstraintCinfo_vtbl);

	constraintData = nullptr;
	priority = hkpConstraintInstance::ConstraintPriority::PRIORITY_PSI;

	this->pivotA[0] = pivotA.x;
	this->pivotA[1] = pivotA.y;
	this->pivotA[2] = pivotA.z;
	this->pivotA[3] = 0.f;

	this->pivotB[0] = pivotB.x;
	this->pivotB[1] = pivotB.y;
	this->pivotB[2] = pivotB.z;
	this->pivotB[3] = 0.f;

	this->rigidBodyA = rigidBodyA;
	this->rigidBodyB = rigidBodyB;

	get_vfunc<_hkConstraintCinfo_CreateConstraintData>(this, 4)(this); // Creates constraintData

	if (constraintData) {
		hkpBallAndSocketConstraintData_setInBodySpace((hkpBallAndSocketConstraintData *)constraintData.val(), NiPointToHkVector(pivotA), NiPointToHkVector(pivotB));
	}
}

bhkGroupConstraint *CreateBallAndSocketConstraint(hkpRigidBody *rigidBodyA, hkpRigidBody *rigidBodyB, NiPoint3 &pivotA, NiPoint3 &pivotB)
{
	hkBallAndSocketConstraintCinfo cinfo(rigidBodyA, rigidBodyB, pivotA, pivotB);
	bhkGroupConstraint *constraint = (bhkGroupConstraint *)Heap_Allocate(sizeof(bhkGroupConstraint));
	bhkGroupConstraint_ctor(constraint, &cinfo);
	constraint->collisionGroup = rigidBodyB->getCollidable()->getCollisionFilterInfo() >> 16;
	return constraint;
}

bhkGroupConstraint *CreateBallAndSocketConstraint2(hkpRigidBody *rigidBodyA, hkpRigidBody *rigidBodyB, NiPoint3 &pivotA, NiPoint3 &pivotB)
{
	hkConstraintCinfo cinfo{};
	cinfo.constraintData = nullptr;
	cinfo.priority = hkpConstraintInstance::ConstraintPriority::PRIORITY_PSI;
	cinfo.rigidBodyA = rigidBodyA;
	cinfo.rigidBodyB = rigidBodyB;

	hkpBallAndSocketConstraintData *constraintData = hkAllocReferencedObject<hkpBallAndSocketConstraintData>();
	hkpBallAndSocketConstraintData_ctor(constraintData);
	cinfo.constraintData = constraintData; // increments refcount
	hkReferencedObject_removeReference(constraintData);

	if (constraintData) {
		hkpBallAndSocketConstraintData_setInBodySpace(constraintData, NiPointToHkVector(pivotA), NiPointToHkVector(pivotB));
	}

	bhkGroupConstraint *constraint = (bhkGroupConstraint *)Heap_Allocate(sizeof(bhkGroupConstraint));
	bhkGroupConstraint_ctor(constraint, &cinfo); // creates the constraint instance from the constraint data in cinfo
	constraint->collisionGroup = rigidBodyB->getCollidable()->getCollisionFilterInfo() >> 16;
	return constraint;
}


namespace RE
{
	/*
	struct hkReferencedObject
	{
		void *vtbl; // 00
		hkUint16 m_memSizeAndFlags; // 08
		hkInt16 m_referenceCount; // 0C
	};

	struct hkpConstraintData : hkReferencedObject
	{
		hkUlong m_userData; // 10
	};
	*/

	class GrabConstraintData : public hkpConstraintData
	{
	public:
		inline void * operator new(hk_size_t nbytes) {
			hkReferencedObject *b = static_cast<hkReferencedObject *>(hkHeapAlloc(static_cast<int>(nbytes)));
			b->m_memSizeAndFlags = static_cast<hkUint16>(nbytes);
			return b;
		}

		inline void operator delete(void *p) {
			hkReferencedObject *b = static_cast<hkReferencedObject *>(p);
			hkGetMemoryRouter().heap().blockFree(p, b->m_memSizeAndFlags);
		}

		inline void * operator new(hk_size_t, void *p) { return p; }
		inline void * operator new[](hk_size_t, void *p) { HK_BREAKPOINT(0); return p; }
		inline void operator delete(void *, void *) { }
		inline void operator delete[](void *, void *) { HK_BREAKPOINT(0); }

		GrabConstraintData() : hkpConstraintData() {}

		void setPivotsInBodySpace(const hkVector4 &pivotA, const hkVector4 &pivotB)
		{
			m_atoms.m_pivots.m_translationA = pivotA;
			m_atoms.m_pivots.m_translationA = pivotB;
		}

		virtual const hkClass *getClassType() const { return 0; }

		virtual void calcContentStatistics(hkStatisticsCollector *collector, const hkClass *cls) const { return; }

		virtual void setMaxLinearImpulse(hkReal maxImpulse)
		{
			m_atoms.m_ballSocket.m_maxImpulse = maxImpulse;
		}

		/// Gets the maximum impulse that can be applied by this constraint.
		virtual hkReal getMaxLinearImpulse() const
		{
			return m_atoms.m_ballSocket.m_maxImpulse;
		}

		/// Choose the body to be notified when the constraint's maximum impulse is breached.
		virtual void setBodyToNotify(int bodyIdx)
		{
			m_atoms.m_ballSocket.m_bodiesToNotify = 1 << bodyIdx;
		}

		/// Returns the index of the body that is notified when the constraint's maximum impulse is breached.
		virtual hkUint8 getNotifiedBodyIndex() const
		{
			return m_atoms.m_ballSocket.m_bodiesToNotify >> 1;
		}

		/// Check consistency of constraint members.
		virtual hkBool isValid() const
		{
			return m_atoms.m_ballSocket.m_solvingMethod != hkpConeLimitConstraintAtom::SolvingMethod::METHOD_STABILIZED || m_atoms.m_setupStabilization.m_enabled;
		}

		/// Get type from this constraint.
		virtual int getType() const
		{
			return hkpConstraintData::ConstraintType::CONSTRAINT_TYPE_CUSTOM;
		}

		/// Sets the solving method for this constraint. Use one of the hkpConstraintAtom::SolvingMethod as a value for method.
		virtual void setSolvingMethod(hkpConstraintAtom::SolvingMethod method)
		{
			if (method == hkpConstraintAtom::SolvingMethod::METHOD_STABILIZED) {
				m_atoms.m_setupStabilization.m_enabled = true;
			}
			else {
				m_atoms.m_setupStabilization.m_enabled = false;
			}

			m_atoms.m_ballSocket.m_solvingMethod = method;
		}

		/// Gets the inertia stabilization factor, returns HK_FAILURE if the factor is not defined for the given constraint.
		virtual hkResult getInertiaStabilizationFactor(hkReal &inertiaStabilizationFactorOut) const
		{
			inertiaStabilizationFactorOut = m_atoms.m_ballSocket.getInertiaStabilizationFactor();
			return hkResult::HK_SUCCESS;
		}

		/// Sets the inertia stabilization factor, return HK_FAILURE if the factor is not defined for the given constraint.
		virtual hkResult setInertiaStabilizationFactor(const hkReal inertiaStabilizationFactorIn)
		{
			m_atoms.m_ballSocket.setInertiaStabilizationFactor(std::clamp(inertiaStabilizationFactorIn, 0.f, 1.f));
			return hkResult::HK_SUCCESS;
		}

		//
		//	Solver interface
		//
		enum
		{
			SOLVER_RESULT_LIN_0 = 0,		// linear constraint 
			SOLVER_RESULT_LIN_1 = 1,		// linear constraint 
			SOLVER_RESULT_LIN_2 = 2,		// linear constraint 
			SOLVER_RESULT_MAX = 3
		};

		struct Runtime
		{
			class hkpSolverResults m_solverResults[3/*VC6 doesn't like the scoping for SOLVER_RESULT_MAX*/];
		};

		inline Runtime *getRuntime(hkpConstraintRuntime *runtime) { return reinterpret_cast<Runtime *>(runtime); }

	public:

		struct Atoms
		{
			struct hkpSetLocalTranslationsConstraintAtom	m_pivots; // 00
			struct hkpSetupStabilizationAtom				m_setupStabilization; // 90
			struct hkpBallSocketConstraintAtom				m_ballSocket; // A0

			Atoms() {	}

			// get a pointer to the first atom
			const hkpConstraintAtom *getAtoms() const { return &m_pivots; }

			int getSizeOfAllAtoms() const { return hkGetByteOffsetInt(this, &m_ballSocket + 1); }
		};

		HK_ALIGN16(struct Atoms m_atoms); // 20

		//
		// Internal functions.
		//
		// hkpConstraintData interface implementations
		virtual void getConstraintInfo(hkpConstraintData::ConstraintInfo &infoOut) const
		{
			GetConstraintInfoFromAtoms(m_atoms.getAtoms(), m_atoms.getSizeOfAllAtoms(), infoOut);
		}

		// hkpConstraintData interface implementations
		virtual void getRuntimeInfo(hkBool wantRuntime, hkpConstraintData::RuntimeInfo &infoOut) const
		{
			if (wantRuntime || m_atoms.m_ballSocket.m_maxImpulse != HK_REAL_MAX) {
				infoOut.m_numSolverResults = 3;
				infoOut.m_sizeOfExternalRuntime = 3 * sizeof(hkpSolverResults);
			}
			else {
				infoOut.m_numSolverResults = 0;
				infoOut.m_sizeOfExternalRuntime = 0;
			}
		}

		/// Access to the solver results in a generic way. Use getRuntimeInfo to get the number of solver results.
		virtual hkpSolverResults *getSolverResults(hkpConstraintRuntime *runtime) { return getRuntime(runtime)->m_solverResults; } // 0D

		/// Initialize the runtime data. The default implementation simply zeros all values.
		virtual void addInstance(hkpConstraintInstance *constraint, hkpConstraintRuntime *runtime, int sizeOfRuntime) const
		{
			if (runtime) {
				memset(runtime, 0, sizeOfRuntime);
			}
		}

	};
}

bhkGroupConstraint *CreateGrabConstraint(hkpRigidBody *rigidBodyA, hkpRigidBody *rigidBodyB, NiPoint3 &pivotA, NiPoint3 &pivotB)
{
	hkConstraintCinfo cinfo{};
	cinfo.constraintData = nullptr;
	cinfo.priority = hkpConstraintInstance::ConstraintPriority::PRIORITY_PSI;
	cinfo.rigidBodyA = rigidBodyA;
	cinfo.rigidBodyB = rigidBodyB;

	RE::GrabConstraintData *constraintData = new RE::GrabConstraintData();

	cinfo.constraintData = constraintData; // increments refcount
	hkReferencedObject_removeReference(constraintData);

	constraintData->setPivotsInBodySpace(NiPointToHkVector(pivotA), NiPointToHkVector(pivotB));

	bhkGroupConstraint *constraint = (bhkGroupConstraint *)Heap_Allocate(sizeof(bhkGroupConstraint));
	bhkGroupConstraint_ctor(constraint, &cinfo); // creates the constraint instance from the constraint data in cinfo
	constraint->collisionGroup = rigidBodyB->getCollidable()->getCollisionFilterInfo() >> 16;
	return constraint;
}
