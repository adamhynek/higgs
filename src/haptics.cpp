#include <algorithm>

#include "hand_broker.h"
#include "haptic_schedule.h"
#include "haptics.h"
#include "math_utils.h"
#include "utils.h"


void HapticsManager::TriggerHapticPulse(float duration)
{
    if (g_openVR && *g_openVR && duration > 0) {
        BSOpenVR *openVR = *g_openVR;
        openVR->TriggerHapticPulse(hand, duration);
    }
}


void HapticsManager::QueueHapticEvent(float startStrength, float endStrength, float duration)
{
    HapticEvent hapticEvent{};
    hapticEvent.startStrength = startStrength;
    hapticEvent.endStrength = endStrength;
    hapticEvent.duration = duration;
    hapticEvent.isNew = true;

    {
        std::scoped_lock lock(eventsLock);
        events.push_back(std::move(hapticEvent));
    }
}

void HapticsManager::QueueHapticPulse(float strength)
{
    QueueHapticEvent(strength, strength, *g_deltaTime * 2.0f);
}

void HapticsManager::Loop()
{
    while (true) {
        std::optional<float> nativeStrength;
        {
            std::scoped_lock lock(eventsLock);

            size_t numEvents = events.size();
            if (numEvents > 0) {
                double currentTime = GetTime();

                // Just play the last event that was added
                HapticEvent &lastEvent = events[numEvents - 1];

                if (lastEvent.isNew) {
                    lastEvent.isNew = false;
                    lastEvent.startTime = currentTime;
                }

                float strength;
                if (lastEvent.duration == 0) {
                    strength = lastEvent.startStrength;
                }
                else {
                    // Simple lerp from start to end strength over duration
                    double elapsedTime = currentTime - lastEvent.startTime;
                    strength = lerp(lastEvent.startStrength, lastEvent.endStrength, min(1.0f, elapsedTime / lastEvent.duration));
                }
                nativeStrength = strength;

                // Cleanup events that are past their duration
                auto end = std::remove_if(events.begin(), events.end(),
                    [currentTime](HapticEvent &evnt) { return currentTime - evnt.startTime >= evnt.duration; }
                );
                events.erase(end, events.end());
            }
        }

        {
            auto phase = HiggsHands::Service().NativePhase();
            const auto physicalHand = hand == BSVRInterface::kControllerHand_Left ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT;
            const auto sample = HiggsHands::Haptics().Sample(physicalHand, HiggsHands::Now(), nativeStrength);
            if (sample.strength > 0) {
                const bool available = g_openVR && *g_openVR;
                if (available) TriggerHapticPulse(sample.strength);
                HiggsHands::Haptics().Submitted(sample, available);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); // TriggerHapticPulse can only be called once every 5ms
    }
}
