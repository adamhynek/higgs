#pragma once
#include "higgshaptics.h"
#include <array>
#include <mutex>
namespace HiggsHands
{
class Broker;
struct HapticOutput
{
    uint64_t serial = 0, atNs = 0;
    uint32_t hand = 0;
    float strength = 0;
};
class HapticSchedule
{
  public:
    explicit HapticSchedule(Broker &broker) : broker_(broker)
    {
    }
    HIGGS_HandStatus Request(uint64_t, const HIGGS_HapticRequest &, uint64_t &, uint64_t, uint64_t operation = 0,
                             bool legacy = false);
    HIGGS_HandStatus Cancel(uint64_t, uint64_t);
    HIGGS_HandStatus Copy(uint64_t, uint64_t, HIGGS_HapticState &, uint64_t);
    HapticOutput Sample(uint32_t, uint64_t);
    void Submitted(const HapticOutput &, bool);
    void Legacy(uint32_t, float, float, float, uint64_t);

  private:
    struct Event
    {
        HIGGS_HapticState state{};
        HIGGS_HapticRequest request{};
        uint64_t operation = 0;
    };
    struct Output
    {
        HapticOutput value{};
        std::array<uint64_t, 256> requests{};
        size_t count = 0;
        bool reported = false;
    };
    void Refresh(uint64_t);
    Broker &broker_;
    std::mutex mutex_;
    std::array<Event, 256> events_{};
    std::array<Output, 2> outputs_{};
    uint64_t next_ = 1, epoch_ = 0;
};
HapticSchedule &Haptics();
} // namespace HiggsHands
