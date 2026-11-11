#pragma once
#include "higgsinput.h"
#include <array>
#include <mutex>
namespace HiggsHands
{
constexpr uint32_t kAuxiliaryInputBase = 0x100, kInputControlSlots = 66;
uint64_t KnownControlMask() noexcept;
uint64_t ActionControlMask() noexcept;
HIGGS_HandStatus ValidateInputCandidate(const HIGGS_InputCandidate &) noexcept;
uint32_t InputControlId(uint32_t brokerButton) noexcept;
uint32_t InputControlSlot(uint32_t brokerButton) noexcept;
class InputFrameStore
{
  public:
    bool Publish(HIGGS_ControllerFrame &, std::array<HIGGS_InputEvent, 96> &, uint32_t &);
    void Epoch(uint64_t);
    HIGGS_HandStatus Copy(uint64_t, HIGGS_InputSnapshot &);
    HIGGS_HandStatus History(const HIGGS_InputFrameQuery &, HIGGS_ControllerFrame *, uint32_t, uint32_t &);
    bool Latest(uint32_t, HIGGS_ControllerFrame &);
    void RetainSuppression(uint32_t, uint64_t frameId, uint64_t mask);
    uint64_t Suppression(uint32_t, uint64_t frameId);

  private:
    void EpochLocked(uint64_t);
    std::mutex mutex_;
    uint64_t epoch_ = 0, nextFrame_ = 1;
    std::array<HIGGS_ControllerFrame, 2> latest_{};
    std::array<HIGGS_ControllerFrame, 128> history_{};
    std::array<std::array<uint64_t, 64>, 2> pressedAt_{};
    uint64_t suppressed_[2]{};
    size_t count_ = 0, cursor_ = 0;
};
InputFrameStore &InputFrames();
uint64_t CurrentControllerFrameId(uint32_t physicalHand) noexcept;
void BeginControllerFrame(HIGGS_ControllerFrame);
void FinishControllerFrame(uint32_t physicalHand, uint64_t &pressed, uint64_t &touched);
} // namespace HiggsHands
