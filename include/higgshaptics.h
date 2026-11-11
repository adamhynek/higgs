#ifndef HIGGS_HAPTICS_H
#define HIGGS_HAPTICS_H
#include "higgshandapi.h"
#ifdef __cplusplus
extern "C"
{
#endif
    enum
    {
        HIGGS_HAPTIC_QUEUED = 1,
        HIGGS_HAPTIC_ACTIVE = 2,
        HIGGS_HAPTIC_COMPLETED = 3,
        HIGGS_HAPTIC_CANCELLED = 4
    };
#pragma pack(push, 8)
    typedef struct HIGGS_HapticRequest
    {
        uint32_t structSize, physicalHand;
        uint64_t epoch, contextKey;
        uint32_t delayMs, durationMs;
        float startStrength, endStrength;
    } HIGGS_HapticRequest;
    typedef struct HIGGS_HapticState
    {
        uint32_t structSize, stage;
        uint64_t requestId, epoch;
        HIGGS_HandClient owner;
        uint32_t physicalHand, submittedSamples, suppressedSamples, unavailableSamples;
        int32_t effectivePriority;
        uint32_t reserved;
        uint64_t dueAtNs, endsAtNs, lastSubmittedAtNs;
    } HIGGS_HapticState;
    typedef struct HIGGS_HapticsInterface
    {
        uint32_t structSize, abiVersion;
        uint64_t capabilities;
        HIGGS_HandStatus(HIGGS_HAND_CALL *RequestHaptic)(HIGGS_HandClient, const HIGGS_HapticRequest *, uint64_t *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CancelHaptic)(HIGGS_HandClient, uint64_t);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyHaptic)(HIGGS_HandClient, uint64_t, HIGGS_HapticState *);
    } HIGGS_HapticsInterface;
#pragma pack(pop)
    /* One coordinated output per physical hand, including all legacy HIGGS calls.
       Higher configured client priority wins; equal-priority envelopes combine by
       maximum strength, independent of submission order. Legacy HIGGS uses the
       configurable client name HIGGS with fallback 0. contextKey!=0 replaces only
       that owner's same-hand/same-context request. No hand/input grant is acquired.
       Delays <=5000ms, durations 1..5000ms, strengths [0,1]. Client/world revocation
       and cancellation remove scheduled effects; an issued hardware pulse cannot be
       recalled. submittedSamples confirms native API submission only, not delivery.
       Queued/active requests bounded256; oldest terminal records may be evicted.
       Exact copied v1; same client/table lifetime and sized-output rules as hand API. */
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryHapticsInterfaceFn)(uint32_t, uint32_t,
                                                                             HIGGS_HapticsInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHapticsInterface(uint32_t, uint32_t, HIGGS_HapticsInterface *);
#ifdef __cplusplus
}
#endif
#endif
