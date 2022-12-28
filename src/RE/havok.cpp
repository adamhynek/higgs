#pragma once

#include "RE/havok.h"
#include "utils.h"

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

	NiPoint3 hkPivotA = pivotA;
	NiPoint3 hkPivotB = pivotB;

	this->pivotA[0] = hkPivotA.x;
	this->pivotA[1] = hkPivotA.y;
	this->pivotA[2] = hkPivotA.z;
	this->pivotA[3] = 0.f;

	this->pivotB[0] = hkPivotB.x;
	this->pivotB[1] = hkPivotB.y;
	this->pivotB[2] = hkPivotB.z;
	this->pivotB[3] = 0.f;

	this->rigidBodyA = rigidBodyA;
	this->rigidBodyB = rigidBodyB;

	get_vfunc<_hkConstraintCinfo_CreateConstraintData>(this, 4)(this); // Creates constraintData

	if (constraintData) {
		hkpBallAndSocketConstraintData_setInBodySpace((hkpBallAndSocketConstraintData *)constraintData.val(), NiPointToHkVector(hkPivotA), NiPointToHkVector(hkPivotB));
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
