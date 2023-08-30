#pragma once

#include "RE/havok.h"
#include "RE/offsets.h"
#include "constraint.h"
#include "utils.h"
#include "skse64/GameRTTI.h"

hkMemoryRouter &hkGetMemoryRouter()
{
	return *(hkMemoryRouter *)(hkUlong)TlsGetValue(*g_havokMemoryRouterTlsIndex);
}

hkUFloat8 &hkUFloat8::operator=(const float &fv)
{
	hkRealTohkUFloat8(*this, fv);
	return *this;
}

namespace RE
{
	template <typename T>
	void hkArray<T>::clear()
	{
		hkArrayUtil::destruct(m_data, m_size, typename hkIsPodType<T>::type());
		m_size = 0;
	}
	template void hkArray<hkpRigidBody *>::clear();

	template <typename T>
	void hkArray<T>::clearAndDeallocate()
	{
		clear();
		if ((m_capacityAndFlags & DONT_DEALLOCATE_FLAG) == 0)
		{
			g_hkContainerHeapAllocator->bufFree(m_data, getCapacity() * hkSizeOfTypeOrVoid<T>::val);
		}
		m_data = HK_NULL;
		m_capacityAndFlags = DONT_DEALLOCATE_FLAG;
	}
	template void hkArray<hkpRigidBody *>::clearAndDeallocate();

	template <typename T>
	void hkArray<T>::pushBack(const T &t)
	{
		if (m_size == getCapacity()) {
			hkArrayUtil__reserveMore(g_hkContainerHeapAllocator, this, sizeof(T));
		}
		m_data[m_size] = t;
		++m_size;
	}
	template void hkArray<hkpRigidBody *>::pushBack(hkpRigidBody *const &t);
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

bhkGroupConstraint *CreateGrabConstraint(hkpRigidBody *rigidBodyA, hkpRigidBody *rigidBodyB, NiTransform &transformA, NiTransform &transformB)
{
	hkConstraintCinfo cinfo{};
	cinfo.constraintData = nullptr;
	cinfo.priority = hkpConstraintInstance::ConstraintPriority::PRIORITY_TOI;
	cinfo.rigidBodyA = rigidBodyA;
	cinfo.rigidBodyB = rigidBodyB;

	GrabConstraintData *constraintData = new GrabConstraintData();

	cinfo.constraintData = constraintData; // increments refcount
	hkReferencedObject_removeReference(constraintData);

	constraintData->setInBodySpace(NiTransformTohkTransform(transformA, false), NiTransformTohkTransform(transformB, false));

	bhkGroupConstraint *constraint = (bhkGroupConstraint *)Heap_Allocate(sizeof(bhkGroupConstraint));
	bhkGroupConstraint_ctor(constraint, &cinfo); // creates the constraint instance from the constraint data in cinfo
	constraint->collisionGroup = rigidBodyB->getCollidable()->getCollisionFilterInfo() >> 16;

	constraintData->setTargetRelativeOrientationOfBodies(NiTransformTohkTransform(transformB, false).m_rotation);
	constraintData->setMotorsActive(constraint->constraint, true);

	return constraint;
}

NiPointer<bhkCharacterController> GetCharacterController(Actor *actor)
{
	ActorProcessManager *process = actor->processManager;
	if (!process) return nullptr;

	MiddleProcess *middleProcess = process->middleProcess;
	if (!middleProcess) return nullptr;

	return *((NiPointer<bhkCharacterController> *) & middleProcess->unk250);
}

NiPointer<bhkCharRigidBodyController> GetCharRigidBodyController(Actor *actor)
{
	NiPointer<bhkCharacterController> controller = GetCharacterController(actor);
	if (!controller) return nullptr;

	return DYNAMIC_CAST(controller, bhkCharacterController, bhkCharRigidBodyController);
}

NiPointer<bhkCharProxyController> GetCharProxyController(Actor *actor)
{
	NiPointer<bhkCharacterController> controller = GetCharacterController(actor);
	if (!controller) return nullptr;

	return DYNAMIC_CAST(controller, bhkCharacterController, bhkCharProxyController);
}
