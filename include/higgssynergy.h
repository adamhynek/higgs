#ifndef HIGGS_SYNERGY_H
#define HIGGS_SYNERGY_H

/* Independent C ABI. This file does not extend or replace IHiggsInterface001. */
#include <stdint.h>

#if defined(_WIN32)
#define HIGGS_SYNERGY_CALL __cdecl
#else
#define HIGGS_SYNERGY_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define HIGGS_SYNERGY_ABI_VERSION 1u
#define HIGGS_SYNERGY_CAP_SCOPED_RESTRICTIONS UINT64_C(1)
#define HIGGS_SYNERGY_MAX_CLIENTS 64u
#define HIGGS_SYNERGY_MAX_LEASES 256u
#define HIGGS_SYNERGY_MIN_LEASE_MS 1u
#define HIGGS_SYNERGY_MAX_LEASE_MS 60000u

typedef uint32_t HIGGS_SynergyStatus;
typedef uint64_t HIGGS_SynergyClient;
typedef uint64_t HIGGS_SynergyLease;

enum {
    HIGGS_SYNERGY_OK = 0,
    HIGGS_SYNERGY_INVALID_ARGUMENT = 1,
    HIGGS_SYNERGY_UNSUPPORTED_VERSION = 2,
    HIGGS_SYNERGY_BUFFER_TOO_SMALL = 3,
    HIGGS_SYNERGY_INVALID_CLIENT = 4,
    HIGGS_SYNERGY_INVALID_LEASE = 5,
    HIGGS_SYNERGY_WRONG_OWNER = 6,
    HIGGS_SYNERGY_NOT_READY = 7,
    HIGGS_SYNERGY_STALE_WORLD = 8,
    HIGGS_SYNERGY_CAPACITY = 9,
    HIGGS_SYNERGY_INTERNAL_ERROR = 10
};

/* Physical hands; these never mean dominant/off hand. */
enum {
    HIGGS_SYNERGY_HAND_LEFT = 1u,
    HIGGS_SYNERGY_HAND_RIGHT = 2u,
    HIGGS_SYNERGY_HAND_BOTH = 3u
};

enum {
    HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS = 1u,
    HIGGS_SYNERGY_SUPPRESS_WEAPON_COLLISION = 2u,
    HIGGS_SYNERGY_FORCE_WEAPON_COLLISION = 4u
};

#pragma pack(push, 8)
typedef struct HIGGS_SynergyClientInfo {
    uint32_t structSize;
    uint32_t reserved; /* Must be zero. */
    char name[64];    /* Copied, nonempty, NUL terminated; descriptive, not authority. */
} HIGGS_SynergyClientInfo;

typedef struct HIGGS_SynergyLeaseRequest {
    uint32_t structSize;
    uint32_t physicalHands;
    uint32_t flags;
    uint32_t durationMs;
    uint64_t expectedWorldGeneration;
} HIGGS_SynergyLeaseRequest;

typedef struct HIGGS_SynergyRenewRequest {
    uint32_t structSize;
    uint32_t durationMs;
    uint64_t expectedWorldGeneration;
    HIGGS_SynergyLease lease;
} HIGGS_SynergyRenewRequest;

typedef struct HIGGS_SynergyLeaseInfo {
    uint32_t structSize;
    uint32_t reserved;
    HIGGS_SynergyLease lease;
    uint64_t worldGeneration;
    uint64_t expiresAtMs; /* Provider monotonic clock; compare to snapshot sampledAtMs. */
} HIGGS_SynergyLeaseInfo;

typedef struct HIGGS_SynergyHandState {
    uint32_t physicalHand;
    uint32_t handInteractionsDisabled;
    uint32_t weaponCollisionDisabled;
    uint32_t forceWeaponCollisionEnabled;
    uint32_t legacyHandDisabled;
    uint32_t legacyWeaponCollisionDisabled;
    uint32_t legacyForceWeaponCollisionEnabled;
    uint32_t scopedFlags;
    uint32_t activeScopedLeases;
} HIGGS_SynergyHandState;

typedef struct HIGGS_SynergyEffectiveState {
    uint32_t structSize;
    uint32_t worldReady;
    uint64_t stateGeneration;
    uint64_t worldGeneration;
    uint64_t sampledAtMs;
    HIGGS_SynergyHandState hands[2]; /* [0] physical left, [1] physical right. */
} HIGGS_SynergyEffectiveState;

typedef struct HIGGS_SynergyInterface {
    uint32_t structSize;
    uint32_t abiVersion;
    uint64_t capabilities;
    uint32_t maxClients;
    uint32_t maxLeases;
    uint32_t minLeaseMs;
    uint32_t maxLeaseMs;
    HIGGS_SynergyStatus (HIGGS_SYNERGY_CALL *RegisterClient)(const HIGGS_SynergyClientInfo*, HIGGS_SynergyClient*);
    HIGGS_SynergyStatus (HIGGS_SYNERGY_CALL *UnregisterClient)(HIGGS_SynergyClient);
    HIGGS_SynergyStatus (HIGGS_SYNERGY_CALL *AcquireLease)(HIGGS_SynergyClient, const HIGGS_SynergyLeaseRequest*, HIGGS_SynergyLeaseInfo*);
    HIGGS_SynergyStatus (HIGGS_SYNERGY_CALL *RenewLease)(HIGGS_SynergyClient, const HIGGS_SynergyRenewRequest*, HIGGS_SynergyLeaseInfo*);
    HIGGS_SynergyStatus (HIGGS_SYNERGY_CALL *ReleaseLease)(HIGGS_SynergyClient, HIGGS_SynergyLease);
    HIGGS_SynergyStatus (HIGGS_SYNERGY_CALL *CopyEffectiveState)(HIGGS_SynergyClient, HIGGS_SynergyEffectiveState*);
} HIGGS_SynergyInterface;
#pragma pack(pop)

/* Resolve this symbol with GetProcAddress on higgs_vr.dll. Only exact v1 is accepted.
   tableSize is the caller's buffer capacity. No output is written on query failure.
   Record outputs need structSize initialized; BUFFER_TOO_SMALL sets its required size.
   Pointers are caller-owned and retained only for this call. Handles are runtime only.
   Stop/join the client's callers, then UnregisterClient before unloading its module. */
typedef HIGGS_SynergyStatus (HIGGS_SYNERGY_CALL *HIGGS_QuerySynergyInterfaceFn)(uint32_t version, uint32_t tableSize, HIGGS_SynergyInterface*);
HIGGS_SynergyStatus HIGGS_SYNERGY_CALL HIGGS_QuerySynergyInterface(uint32_t version, uint32_t tableSize, HIGGS_SynergyInterface* outInterface);

#ifdef __cplusplus
}
#endif
#endif
