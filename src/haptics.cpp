#include <algorithm>

#include "hand_broker.h"
#include "haptic_schedule.h"
#include "haptics.h"
#include "math_utils.h"
#include "utils.h"

void HapticsManager::TriggerHapticPulse(float duration)
{
    if (g_openVR && *g_openVR && duration > 0)
    {
        BSOpenVR *openVR = *g_openVR;
        openVR->TriggerHapticPulse(hand, duration);
    }
}

void HapticsManager::QueueHapticEvent(float startStrength, float endStrength, float duration)
{
    HiggsHands::Haptics().Legacy(hand == BSVRInterface::kControllerHand_Left ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT,
                                 startStrength, endStrength, duration, HiggsHands::Now());
}

void HapticsManager::QueueHapticPulse(float strength)
{
    QueueHapticEvent(strength, strength, *g_deltaTime * 2.0f);
}

void HapticsManager::Loop()
{
    while (true)
    {
        {
            auto phase = HiggsHands::Service().NativePhase();
            const auto sample = HiggsHands::Haptics().Sample(
                hand == BSVRInterface::kControllerHand_Left ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT, HiggsHands::Now());
            if (sample.strength > 0)
            {
                const bool available = g_openVR && *g_openVR;
                if (available)
                    TriggerHapticPulse(sample.strength);
                HiggsHands::Haptics().Submitted(sample, available);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
