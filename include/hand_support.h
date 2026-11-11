#pragma once
#include "higgsphysics.h"
#include <array>
#include <mutex>
namespace HiggsHands
{
class Broker;
class SupportService
{
  public:
    explicit SupportService(Broker &broker) : broker_(broker)
    {
    }
    using Driver = HIGGS_HandStatus (*)(HIGGS_SupportState &, bool ending, void *);
    void Install(Driver, void *, uint64_t);
    uint64_t Capabilities();
    HIGGS_HandStatus Request(uint64_t, const HIGGS_SupportRequest &, uint64_t &, uint64_t);
    HIGGS_HandStatus Update(uint64_t, uint64_t, const HIGGS_SupportUpdate &);
    HIGGS_HandStatus End(uint64_t, uint64_t, uint64_t);
    HIGGS_HandStatus Copy(uint64_t, uint64_t, HIGGS_SupportState &);
    void Drain(uint64_t, uint32_t endingHand = 0);
    bool Goal(uint32_t, HIGGS_SupportState &, uint64_t);
    void Applied(uint64_t, uint64_t);
    void Break(uint64_t, uint32_t);
    void BreakHand(uint32_t, uint32_t);

  private:
    struct Slot
    {
        HIGGS_SupportState state{};
        uint64_t deadline = 0;
        bool ownGrant = false;
    };
    Slot *Find(uint64_t);
    Broker &broker_;
    std::mutex mutex_;
    std::array<Slot, 64> slots_{};
    Driver driver_ = nullptr;
    void *context_ = nullptr;
    uint64_t capabilities_ = 0;
};
SupportService &Supports();
void InitializeNativeSupport(void *messaging, uint32_t pluginHandle);
void DrainNativeSupport();
bool CopySupportTarget(bool, float[3], float[9], float &);
void ConfirmSupportTarget(bool);
void PrepareSupportRelease(bool);
} // namespace HiggsHands
