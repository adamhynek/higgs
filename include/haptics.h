#pragma once

#include <vector>
#include <mutex>

#include "skse64/GameVR.h"


struct HapticsManager
{
    struct HapticEvent
    {
        float startStrength;
        float endStrength;
        double duration;
        double startTime;
        bool isNew;
    };

    HapticsManager(BSVRInterface::BSControllerHand hand) :
        hand(hand),
        thread(&HapticsManager::Loop, this)
    {}

    BSVRInterface::BSControllerHand hand;
    std::vector<HapticEvent> events;
    std::mutex eventsLock;
    std::thread thread;

    void TriggerHapticPulse(float duration);
    void QueueHapticEvent(float startStrength, float endStrength, float duration);
    void QueueHapticPulse(float duration);
    void Loop();
};
