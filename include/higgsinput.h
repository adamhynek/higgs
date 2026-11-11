#ifndef HIGGSINPUT_H
#define HIGGSINPUT_H
#include "higgshandapi.h"
#ifdef __cplusplus
extern "C" {
#endif
#pragma pack(push, 8)

#define HIGGS_INPUT_ABI_VERSION 2u
#define HIGGS_INPUT_CAP_FRAMES UINT64_C(1)
#define HIGGS_INPUT_CAP_OBSERVATIONS UINT64_C(2)
#define HIGGS_INPUT_CAP_PRESS_CANDIDATES UINT64_C(4)
#define HIGGS_INPUT_CAP_DEFERRED_DISCARD UINT64_C(8)
#define HIGGS_INPUT_CAP_SESSION_GRANT UINT64_C(16)
    enum
    {
        HIGGS_CONTROL_SYSTEM = 0,
        HIGGS_CONTROL_MENU = 1,
        HIGGS_CONTROL_GRIP = 2,
        HIGGS_CONTROL_DPAD_LEFT = 3,
        HIGGS_CONTROL_DPAD_UP = 4,
        HIGGS_CONTROL_DPAD_RIGHT = 5,
        HIGGS_CONTROL_DPAD_DOWN = 6,
        HIGGS_CONTROL_A = 7,
        HIGGS_CONTROL_PROXIMITY = 31,
        HIGGS_CONTROL_AXIS0 = 32,
        HIGGS_CONTROL_TRIGGER = 33,
        HIGGS_CONTROL_AXIS2 = 34,
        HIGGS_CONTROL_AXIS3 = 35,
        HIGGS_CONTROL_AXIS4 = 36
    };
    enum
    {
        HIGGS_CONTROL_OBSERVABLE = 1,
        HIGGS_CONTROL_ACTIONABLE = 2,
        HIGGS_CONTROL_EFFECTIVE_GRIP = 4,
        HIGGS_CONTROL_RETURNED_STATE_MASK = 8
    };
    enum
    {
        HIGGS_RAW_PRESS = 1,
        HIGGS_RAW_RELEASE = 2,
        HIGGS_RAW_TOUCH = 3,
        HIGGS_RAW_UNTOUCH = 4,
        HIGGS_RAW_AXIS = 5,
        HIGGS_RAW_HELD = 6
    };
    enum
    {
        HIGGS_INPUT_ACTIONABLE = 1,
        HIGGS_INPUT_EFFECTIVE_SEMANTIC = 2
    };
    enum
    {
        HIGGS_INPUT_FRAME_READY = 0,
        HIGGS_INPUT_FRAME_INVALID = 1,
        HIGGS_INPUT_FRAME_DISCONNECTED = 2
    };
    enum
    {
        HIGGS_AXIS_NONE = 0,
        HIGGS_AXIS_TRACKPAD = 1,
        HIGGS_AXIS_JOYSTICK = 2,
        HIGGS_AXIS_TRIGGER = 3,
        HIGGS_AXIS_UNKNOWN = 4
    };
    enum
    {
        HIGGS_INPUT_GRIP_USES_TOUCH = 1,
        HIGGS_INPUT_NO_HARDWARE_TIME = 2,
        HIGGS_INPUT_BASELINE = 4
    };
    enum
    {
        HIGGS_GESTURE_IMMEDIATE = 0,
        HIGGS_GESTURE_HOLD = 1,
        HIGGS_GESTURE_MULTI_TAP = 2
    };
    enum
    {
        HIGGS_DEFAULT_DISCARD_AFTER_SELECTION = 1
    };
    enum
    {
        HIGGS_RECOGNITION_PROPOSED = 0,
        HIGGS_RECOGNITION_WAITING = 1,
        HIGGS_RECOGNITION_MATCHED = 2,
        HIGGS_RECOGNITION_EXPIRED = 3,
        HIGGS_RECOGNITION_CANCELLED = 4
    };
    typedef struct HIGGS_ControlInfo
    {
        uint32_t id, flags;
        char name[24];
    } HIGGS_ControlInfo;
    typedef struct HIGGS_ControllerFrame
    {
        uint32_t structSize, physicalHand, deviceIndex, packetNumber, axisValidMask, status, flags, reserved;
        uint64_t epoch, frameId, observedAtNs, handObservationId, pressed, touched;
        float axes[5][2];
        uint32_t axisTypes[5]; /* public OpenVR axis type; unknown is not stationary */
    } HIGGS_ControllerFrame;
    typedef struct HIGGS_InputFrameQuery
    {
        uint32_t structSize, physicalHands;
        uint64_t epoch, afterFrame;
    } HIGGS_InputFrameQuery;
    typedef struct HIGGS_InputSnapshot
    {
        uint32_t structSize, reserved;
        uint64_t epoch;
        HIGGS_ControllerFrame hands[2]; /* latest retained pair; independent sample times */
    } HIGGS_InputSnapshot;
    typedef struct HIGGS_InputEvent
    {
        uint32_t structSize, physicalHand, controlId, kind, flags, axisIndex;
        uint64_t inputId, heldDurationNs;
        HIGGS_ControllerFrame frame;
        HIGGS_HandSnapshot snapshot; /* broker context for THIS recognition */
    } HIGGS_InputEvent;
    typedef void(HIGGS_HAND_CALL *HIGGS_InputRecognizerCallback)(const HIGGS_InputEvent *, void *);
    typedef struct HIGGS_InputRecognizerSpec
    {
        uint32_t structSize, physicalHands, eventMask, reserved; /* bit(kind-1); zero selects all */
        uint64_t controlMask;
        HIGGS_InputRecognizerCallback callback;
        void *context;
    } HIGGS_InputRecognizerSpec;
    typedef struct HIGGS_InputCandidate
    {
        uint32_t structSize, gesture, defaultPolicy, deadlineMs;
        uint32_t holdMs, maxTapMs, tapGapMs, tapCount;
        HIGGS_HandGrant existingGrant; /* optional assigned executor's current session grant */
        HIGGS_ActionCandidate candidate;
    } HIGGS_InputCandidate;
    typedef struct HIGGS_InputRecognition
    {
        uint32_t structSize, stage, gesture, matchedTaps;
        uint64_t requestId, epoch, inputId, deadlineAtNs, recognizedAtNs;
        HIGGS_ControllerFrame initialFrame, recognizedFrame;
    } HIGGS_InputRecognition;
    typedef struct HIGGS_InputInterface
    {
        uint32_t structSize, abiVersion;
        uint64_t capabilities;
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyControlCatalog)(HIGGS_ControlInfo *, uint32_t, uint32_t *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopySnapshot)(HIGGS_HandClient, HIGGS_InputSnapshot *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyFrames)(HIGGS_HandClient, const HIGGS_InputFrameQuery *,
                                                      HIGGS_ControllerFrame *, uint32_t, uint32_t *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *RegisterRecognizer)(HIGGS_HandClient, const HIGGS_InputRecognizerSpec *,
                                                              HIGGS_HandRegistration *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *SubmitCandidate)(HIGGS_HandClient, const HIGGS_InputCandidate *,
                                                           HIGGS_HandRequestId *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyRecognition)(HIGGS_HandClient, HIGGS_HandRequestId,
                                                           HIGGS_InputRecognition *);
    } HIGGS_InputInterface;
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryInputInterfaceFn)(uint32_t, uint32_t, HIGGS_InputInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryInputInterface(uint32_t, uint32_t, HIGGS_InputInterface *);
/* Exact v2 copied records. Remove recognizers through HandInterface.RemoveRegistration;
 * it shares existing callback drainage. Callback events with inputId0/without
 * ACTIONABLE are immutable observations, not authority to execute gameplay.
 * Candidates share the existing broker priority winner. Grip/trigger retain their
 * effective native semantics; other controls use the fixed OpenVR catalogue.
 * Deferred hold/multi-tap selects/reserves the initial press, then invokes its one
 * executor on a match. Deadline/cancel discards defaults through release; no input
 * replay is supported. System/proximity are observation-only. Mask coverage is the
 * returned OpenVR controller state and HIGGS defaults, not every engine/legacy path.
 * In particular full trigger masking is NOT a draw-only/unsheath policy.
 * Caller input/output storage must be accessible, stable and nonoverlapping. */
#ifdef __cplusplus
static_assert(sizeof(HIGGS_ControllerFrame) == 144, "controller frame layout changed");
static_assert(sizeof(HIGGS_InputInterface) == 64, "extended input table changed (x64)");
#endif

enum
    {
        HIGGS_SEQUENCE_HELD = 1,
        HIGGS_SEQUENCE_RELEASED = 2,
        HIGGS_SEQUENCE_CANCELLED = 3
    };
    enum
    {
        HIGGS_ROUTE_NATIVE_GRIP = 1,
        HIGGS_ROUTE_ORDERED_CANDIDATES = 2,
        HIGGS_ROUTE_TERMINALS = 4,
        HIGGS_ROUTE_NATIVE_TRIGGER_GRAB = 8
    };
    typedef struct HIGGS_InputSequence
    {
        uint32_t structSize, phase, physicalHand, button, nativeAction, reason;
        uint64_t epoch, pressInputId, terminalInputId, requestId, executor, correlation, actionId;
    } HIGGS_InputSequence;
    typedef struct HIGGS_InputRoutesInterface
    {
        uint32_t structSize, abiVersion;
        uint64_t capabilities;
        HIGGS_HandStatus(HIGGS_HAND_CALL *SubmitOrderedCandidate)(HIGGS_HandClient, const HIGGS_ActionCandidate *,
                                                                  uint64_t stableTargetKey, uint64_t *requestId);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopySequence)(HIGGS_HandClient, uint64_t pressInputId,
                                                        HIGGS_InputSequence *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CheckSelectedAction)(HIGGS_HandClient, uint64_t requestId,
                                                               uint64_t epoch, uint32_t physicalHands,
                                                               uint64_t *correlation);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyClientLabel)(HIGGS_HandClient, uint64_t *publicLabel);
    } HIGGS_InputRoutesInterface;
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryInputRoutesInterfaceFn)(uint32_t, uint32_t,
                                                                                 HIGGS_InputRoutesInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryInputRoutesInterface(uint32_t, uint32_t,
                                                                     HIGGS_InputRoutesInterface *);
/* actionId is the provider's persistent semantic key. Provide a stableTargetKey
   to rank different targets of that action. INI provider/actionId priorities
   use decimal IDs. CheckSelectedAction is available to the selected executor
   and its authorized recognizer while the grant remains valid. Its correlation
   matches the opaque grant field in observed decisions and hand records and
   cannot authorize mutations. CopySequence is an observation: it preserves
   the press/selection through grant release, expires after 256 later terminal
   records and gives no new permission. A native selection or acceptance into HIGGS's
   leeway window does not confirm an object was subsequently grabbed. Cancel has terminalInputId zero. Targeted
   INPUT_RELEASED/CANCELLED events reach the selected executor owner only. Their
   result.inputId is the original press. CopyClientLabel returns a public label
   for this live registration for routing, never a credential or durable key. */

#define HIGGS_INPUT_CONTEXT_ABI_VERSION 1u
enum HIGGS_InputContextKind { HIGGS_CONTEXT_GAMEPLAY = 1, HIGGS_CONTEXT_UI = 2 };
enum HIGGS_InputContextFlags { HIGGS_CONTEXT_DRAINING = 1u, HIGGS_CONTEXT_NO_REPLAY = 2u };
typedef struct HIGGS_InputContextHand {
    uint64_t frameId, drainPressed, drainTouched;
} HIGGS_InputContextHand;
typedef struct HIGGS_InputContextState {
    uint32_t structSize, kind, activeMenus, flags;
    uint64_t epoch, generation, changedAtNs;
    HIGGS_InputContextHand hands[2];
} HIGGS_InputContextState;
typedef struct HIGGS_InputContextFrame {
    uint32_t structSize, physicalHand, kind, flags;
    uint64_t epoch, frameId, generation, drainPressed, drainTouched;
} HIGGS_InputContextFrame;
typedef void(HIGGS_HAND_CALL *HIGGS_InputContextCallback)(const HIGGS_InputContextState *previous,
    const HIGGS_InputContextState *current, void *context);
typedef struct HIGGS_InputContextInterface {
    uint32_t structSize, abiVersion;
    uint64_t capabilities; /* bit0: menu handoff, bit1: frame association, bit2: removable events */
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopyContext)(HIGGS_HandClient, HIGGS_InputContextState *);
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopyFrameContext)(HIGGS_HandClient, uint64_t frameId, HIGGS_InputContextFrame *);
    HIGGS_HandStatus(HIGGS_HAND_CALL *Subscribe)(HIGGS_HandClient, HIGGS_InputContextCallback, void *, HIGGS_HandRegistration *);
} HIGGS_InputContextInterface;
typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryInputContextInterfaceFn)(uint32_t, uint32_t, HIGGS_InputContextInterface *);
HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryInputContextInterface(uint32_t, uint32_t, HIGGS_InputContextInterface *);
/* Native menu events are authoritative. UI is an input context, not a pause claim.
 * Raw observations continue without gameplay authority. Existing physical use and
 * real-time expiry continue; new gameplay admission is suspended. Held controls
 * drain through release in both directions, never replay. Frame associations are
 * bounded to the last128 controller samples. Events run serially on the transition
 * thread outside the state mutex. Remove/drain via HandInterface.RemoveRegistration.
 * This describes returned controller/HIGGS routing, not all engine input paths. */
#ifdef __cplusplus
static_assert(sizeof(HIGGS_InputContextState) == 88, "context state layout");
static_assert(sizeof(HIGGS_InputContextFrame) == 56, "context frame layout");
static_assert(sizeof(HIGGS_InputContextInterface) == 40, "context table layout");
#endif

enum
    {
        HIGGS_DRAW_EITHER = 1,
        HIGGS_DRAW_LEFT = 2,
        HIGGS_DRAW_RIGHT = 4,
        HIGGS_DRAW_DUAL_CAST = 8,
        HIGGS_DRAW_ALL_PATHS = 15
    };
    enum
    {
        HIGGS_DRAW_DEFAULT = 0,
        HIGGS_DRAW_ALLOW = 1,
        HIGGS_DRAW_SUPPRESS = 2
    };
    typedef struct HIGGS_TriggerDrawPolicy
    {
        uint32_t structSize, decision, durationMs, reserved;
        uint64_t epoch;
    } HIGGS_TriggerDrawPolicy;
    typedef struct HIGGS_DefaultCommandInterface
    {
        uint32_t structSize, abiVersion;
        HIGGS_HandStatus(HIGGS_HAND_CALL *AttachDrawBoundary)(HIGGS_HandClient, uint32_t verifiedPaths,
                                                              uint64_t *registration);
        HIGGS_HandStatus(HIGGS_HAND_CALL *DetachDrawBoundary)(HIGGS_HandClient, uint64_t registration);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyDrawCoverage)(uint32_t *paths);
        HIGGS_HandStatus(HIGGS_HAND_CALL *SetTriggerDrawPolicy)(HIGGS_HandClient, const HIGGS_TriggerDrawPolicy *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *EvaluateTriggerDraw)(HIGGS_HandClient, uint64_t registration,
                                                               uint32_t path, uint32_t *decision);
    } HIGGS_DefaultCommandInterface;
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryDefaultCommandInterface(uint32_t, uint32_t,
                                                                        HIGGS_DefaultCommandInterface *);
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryDefaultCommandInterfaceFn)(
        uint32_t, uint32_t, HIGGS_DefaultCommandInterface *);
/* Optional engine-side adapter, with all four verified trigger-draw boundaries.
   The adapter calls Evaluate at the semantic command and executes the original
   operation on error or DEFAULT. No controller state is changed. HIGGS has no
   built-in adapter. A registered client sets one epoch-bound, renewable policy
   for 1..60000 ms, or DEFAULT/zero duration to remove it. Provider priority and
   stable provider name select between simultaneous policies. Detaching drops all
   associated policies. A replacement adapter does not inherit them. */

#define HIGGS_NATIVE_REGIONS_ABI_VERSION 1u
#define HIGGS_NATIVE_REGION_COUNT 3u
enum { HIGGS_REGION_MOUTH = 1u, HIGGS_REGION_RIGHT_SHOULDER = 2u,
       HIGGS_REGION_LEFT_SHOULDER = 4u, HIGGS_REGION_ALL = 7u };
enum { HIGGS_REGION_SOURCE_VALID = 1u };
enum { HIGGS_NATIVE_REGION_CAP_DETECTOR = 1u, HIGGS_NATIVE_REGION_CAP_COPIED_FRAME = 2u };
typedef struct HIGGS_NativeRegionSphere {
    float worldCenter[3], worldRadius;
} HIGGS_NativeRegionSphere;
typedef struct HIGGS_NativeRegionFrame {
    uint32_t structSize, physicalHand, flags, reserved;
    uint64_t epoch, sampleId, observedAtNs, sourceGeneration, handGeneration;
    float hmdPosition[3], hmdRotation[9], hmdScale;
    float handPosition[3], palmPosition[3];
    HIGGS_NativeRegionSphere spheres[HIGGS_NATIVE_REGION_COUNT];
} HIGGS_NativeRegionFrame;
typedef struct HIGGS_NativeRegionResult {
    uint32_t structSize, insideMask;
    uint64_t epoch, sampleId;
} HIGGS_NativeRegionResult;
typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_NativeRegionDetector)(
    const HIGGS_NativeRegionFrame*, HIGGS_NativeRegionResult*, void*);
typedef struct HIGGS_NativeRegionsInterface {
    uint32_t structSize, abiVersion;
    uint64_t capabilities;
    HIGGS_HandStatus(HIGGS_HAND_CALL *RegisterDetector)(HIGGS_HandClient,
        HIGGS_NativeRegionDetector, void*, HIGGS_HandRegistration*);
    HIGGS_HandStatus(HIGGS_HAND_CALL *RemoveDetector)(HIGGS_HandClient, HIGGS_HandRegistration);
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopyFrame)(HIGGS_HandClient, uint32_t, HIGGS_NativeRegionFrame*);
} HIGGS_NativeRegionsInterface;
/* Optional synchronous spatial-only callback, at most one active detector.
   Negotiate at PostPostLoad or later. No link or runtime Body Slots dependency.
   Coordinates/radii are Skyrim world units; rotations are row-major. HMD scale
   has already been applied to configured offsets, never to the world radii.
   Sphere order: mouth, right shoulder, left shoulder. Mouth probes palmPosition;
   shoulders probe handPosition. Native legacy containment is strict distance <
   radius. OK + exact epoch/sample and a valid mask handles the entire frame,
   including an all-outside result. Any other status/malformed result selects
   only native fallback. SOURCE_VALID means finite current-phase source data;
   it makes no tracking-freshness promise beyond this native update.
   Callback/context survive until successful RemoveDetector or hand-client
   Unregister. Those calls drain callbacks outside callbacks; callback reentry
   returns Pending on removal. Callbacks must not call HIGGS, execute gameplay,
   retain frame pointers, block, or infer input/action authority from proximity.
   sourceGeneration changes on actual HMD node replacement, configured local
   offsets/radii or HMD scale changes; ordinary head movement does not change it.
   epoch separately identifies world/load scope. Registration survives world/load
   changes; copied source frames do not.
   CopyFrame is last-observed evidence only, not a detector-invocation API. */
const HIGGS_NativeRegionsInterface* HIGGS_HAND_CALL HIGGS_QueryNativeRegionsInterface(uint32_t, uint32_t);
typedef const HIGGS_NativeRegionsInterface*(HIGGS_HAND_CALL *HIGGS_QueryNativeRegionsInterfaceFn)(uint32_t, uint32_t);

#pragma pack(pop)
#ifdef __cplusplus
}
#endif
#endif
