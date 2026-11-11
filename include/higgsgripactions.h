#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One fresh physical grip press (0 = right hand, 1 = left). Offer reads the
 * client's own cached context and returns a chosen action ID, or zero to pass.
 * Selected runs once on the controller thread before HIGGS's native grab.
 * Both callbacks must return promptly. Do not retain an engine pointer, mutate
 * inventory on the controller thread, or treat selection as proof of execution. */
typedef uint64_t (*HiggsGripOffer)(uint32_t isLeft, void* context);
typedef void (*HiggsGripSelected)(uint32_t isLeft, uint64_t action, void* context);

enum { HIGGS_GRIP_OK, HIGGS_GRIP_INVALID, HIGGS_GRIP_CONFLICT, HIGGS_GRIP_BUSY, HIGGS_GRIP_FULL };

typedef struct HiggsGripActions001 {
    /* Unique stable lowercase ASCII ID, up to 63 letters, digits, dots, underscores or hyphens.
     * The player may rank it under [GripActions] in higgs_vr.ini. Register at PostPostLoad.
     * On failure token is unchanged. */
    uint32_t (*Register)(const char* id, HiggsGripOffer offer, HiggsGripSelected selected,
                         void* context, uint64_t* token);
    /* Deactivates the registration immediately. If BUSY retry until OK before
     * freeing callback state or unloading. A running Selected may finish. */
    uint32_t (*Remove)(uint64_t token);
} HiggsGripActions001;

typedef const HiggsGripActions001* (*HiggsGetGripActions001)(void);
const HiggsGripActions001* HIGGS_GetGripActions001(void);

#ifdef __cplusplus
}
namespace HiggsGrip {
    uint64_t Epoch();
    void Invalidate();
    void SetLoading(bool loading);
    bool Select(bool isLeft, uint64_t epoch);
}
#endif
