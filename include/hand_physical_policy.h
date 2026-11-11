#pragma once
#include "higgsphysicalpolicy.h"
#include <array>
#include <mutex>
namespace HiggsHands
{
class Broker;
class PhysicalPolicyService
{
  public:
    explicit PhysicalPolicyService(Broker &broker) : broker_(broker)
    {
    }
    using Driver = HIGGS_HandStatus (*)(HIGGS_PhysicalPolicyState &, bool, void *);
    void Install(Driver, void *, uint64_t);
    uint64_t Capabilities();
    HIGGS_HandStatus Request(uint64_t, const HIGGS_PhysicalPolicyRequest &, uint64_t &, uint64_t);
    HIGGS_HandStatus Update(uint64_t, uint64_t, uint64_t, const HIGGS_PhysicalPolicyRequest &, uint64_t);
    HIGGS_HandStatus End(uint64_t, uint64_t, uint64_t);
    HIGGS_HandStatus Copy(uint64_t, uint64_t, HIGGS_PhysicalPolicyState &);
    void Drain(uint64_t, uint32_t endingHand = 0);
    void EndHeld(uint32_t);
    uint32_t GrabMode(uint32_t, uint64_t);
    bool SuppressFingers(uint32_t, uint64_t);

  private:
    struct Slot
    {
        HIGGS_PhysicalPolicyState state{};
    };
    Slot *Find(uint64_t);
    HIGGS_HandStatus Validate(uint64_t, const HIGGS_PhysicalPolicyRequest &, uint64_t);
    bool Alive(const Slot &, uint64_t);
    bool Conflict(const HIGGS_PhysicalPolicyRequest &, uint64_t except = 0);
    Broker &broker_;
    std::mutex mutex_;
    std::array<Slot, 64> slots_{};
    Driver driver_ = nullptr;
    void *context_ = nullptr;
    uint64_t capabilities_ = 0;
};
PhysicalPolicyService &PhysicalPolicies();
void MaintainNativePhysicalPolicies();
void PreparePhysicalPolicyRelease(bool);
bool PhysicsGrabPreference(bool, bool);
bool SuppressHiggsFingers(bool);
} // namespace HiggsHands
