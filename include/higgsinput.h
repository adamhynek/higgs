#ifndef HIGGS_INPUT_EXTENDED_H
#define HIGGS_INPUT_EXTENDED_H
#include "higgshandapi.h"
#define HIGGS_INPUT_ABI_VERSION 2u
#define HIGGS_INPUT_CAP_FRAMES UINT64_C(1)
#define HIGGS_INPUT_CAP_OBSERVATIONS UINT64_C(2)
#define HIGGS_INPUT_CAP_PRESS_CANDIDATES UINT64_C(4)
#define HIGGS_INPUT_CAP_DEFERRED_DISCARD UINT64_C(8)
#define HIGGS_INPUT_CAP_SESSION_GRANT UINT64_C(16)
#ifdef __cplusplus
extern "C"
{
#endif
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
#pragma pack(push, 8)
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
#pragma pack(pop)
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
}
static_assert(sizeof(HIGGS_ControllerFrame) == 144, "controller frame layout changed");
static_assert(sizeof(HIGGS_InputInterface) == 64, "extended input table changed (x64)");
#endif
#endif
