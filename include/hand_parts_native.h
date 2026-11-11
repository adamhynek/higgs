#pragma once
#include <cstdint>
class hkpRigidBody;
struct bhkWorld;
namespace HiggsHands {
void BeginNativeHandPartsStep(bhkWorld*,float dt);
void ApplyNativeHandPartsStep(bhkWorld*,float dt);
void MaintainNativeHandParts();
bool ResolveNativeHandPart(const hkpRigidBody*,uint32_t& physicalHand,uint32_t& partId,uint64_t& set,uint64_t& revision);
void RecordNativeHandPartContact(const hkpRigidBody*,uint64_t epoch,uint64_t contactSequence);
}
