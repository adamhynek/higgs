#pragma once
#include "hand_input_extended.h"
#include "higgsinputcontext.h"
#include "higgsinputroutes.h"
#include "higgscommandpolicy.h"
#include "higgsgrantmutation.h"
#include "higgsauthorizeddispatch.h"
#include "higgshandapi.h"
#include "higgshandoperations.h"
#include "higgshanduse.h"
#include "higgshandusetransition.h"
#include "higgshanduseproposals.h"
#include "higgstransactions.h"
#include "higgsnativeregions.h"
#include <array>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace HiggsHands
{
using Clock = uint64_t;
// Actual production broker; no engine or SKSE dependency. All native mutation is
// routed through DrainPhysical at the native HIGGS main-update phase.
class Broker
{
  public:
    HIGGS_HandStatus Register(const HIGGS_HandClientInfo &, HIGGS_HandClient &);
    HIGGS_HandStatus Unregister(HIGGS_HandClient);
    HIGGS_HandStatus Copy(HIGGS_HandClient, HIGGS_HandSnapshot &, Clock);
    HIGGS_HandStatus Recognizer(HIGGS_HandClient, HIGGS_RecognizerCallback, void *, uint64_t &);
    HIGGS_HandStatus Executor(HIGGS_HandClient, HIGGS_ExecutorCallback, void *, uint64_t &);
    HIGGS_HandStatus Events(HIGGS_HandClient, HIGGS_HandEventCallback, void *, uint64_t &);
    HIGGS_HandStatus TransferExecutor(uint64_t, HIGGS_TransferExecutorCallback, void *, uint64_t &);
    bool InvokeTransfer(uint64_t, uint64_t, const HIGGS_TransferExecution &);
    bool InTransferCallback(uint64_t, uint64_t);
    bool ClientValid(uint64_t);
    bool ExecutorValid(uint64_t, uint64_t, uint32_t);
    HIGGS_HandStatus ReserveDomain(uint64_t, uint64_t, uint32_t, uint64_t, uint64_t, uint32_t, HIGGS_ActionRequest &,
                                   uint64_t);
    bool DomainAlive(uint64_t, uint64_t, uint64_t);
    HIGGS_HandStatus ReserveEffectAuthority(uint64_t, uint64_t, uint32_t, HIGGS_ActionRequest &, uint64_t,
                                          uint64_t dependencyGrant=0, uint32_t dependencyHands=0);
    HIGGS_HandStatus RenewEffectAuthority(uint64_t, uint64_t, uint32_t, uint64_t);
    HIGGS_HandStatus ClaimActuator(uint64_t request, uint32_t kind);
    void UnclaimActuator(uint64_t request);
    bool EnterDomainMutation(uint64_t request, bool cleanup);
    void ExitDomainMutation(uint64_t request);
    void DomainResult(uint64_t, uint32_t, uint32_t, uint32_t = 0, uint32_t = 0);
    void RetainRequest(uint64_t, bool);
    HIGGS_HandStatus CancelNativeRequest(uint64_t, uint64_t);
    HIGGS_HandStatus Remove(HIGGS_HandClient, uint64_t);
    HIGGS_HandStatus Submit(HIGGS_HandClient, const HIGGS_ActionCandidate &, uint64_t &, Clock, uint64_t stableTarget = 0);
    void AcceptNativePress(uint32_t hand,uint32_t button);
    HIGGS_HandStatus CopySequence(uint64_t, uint64_t, HIGGS_InputSequence &);
    HIGGS_HandStatus CheckSelectedAction(uint64_t, uint64_t, uint64_t, uint32_t, uint64_t &, Clock);
    HIGGS_HandStatus ObservedSnapshot(uint64_t, HIGGS_HandSnapshot &, Clock);
    HIGGS_HandStatus ObservedDecision(uint64_t, uint64_t, HIGGS_InputDecision &);
    uint64_t ClientLabel(uint64_t);
    uint64_t ClientForLabel(uint64_t);
    HIGGS_HandStatus AttachDraw(uint64_t,uint32_t,uint64_t&);
    HIGGS_HandStatus DetachDraw(uint64_t,uint64_t);
    uint32_t DrawCoverage();
    HIGGS_HandStatus DrawPolicy(uint64_t,const HIGGS_TriggerDrawPolicy&,Clock);
    HIGGS_HandStatus EvaluateDraw(uint64_t,uint64_t,uint32_t,uint32_t&,Clock);
    HIGGS_HandStatus Report(HIGGS_HandClient, const HIGGS_ActionResult &);
    HIGGS_HandStatus Result(HIGGS_HandClient, uint64_t, HIGGS_ActionResult &);
    HIGGS_HandStatus Decision(HIGGS_HandClient, uint64_t, HIGGS_InputDecision &);
    HIGGS_HandStatus Release(HIGGS_HandClient, uint64_t);
    HIGGS_HandStatus Physical(HIGGS_HandClient, const HIGGS_PhysicalRequest &, uint64_t &, Clock);
    void ObserveWorld(uintptr_t identity, bool loading, Clock);
    void Publish(const std::array<HIGGS_HandRecord, 2> &, Clock);
    // Returns true when this raw state belongs to a consumed press (including up).
    bool Input(uint32_t hand, uint32_t button, bool down, Clock, bool *nativeContinuation = nullptr);
    HIGGS_HandStatus InputRecognizer(uint64_t, const HIGGS_InputRecognizerSpec &, uint64_t &);
    HIGGS_HandStatus SubmitInputCandidate(uint64_t, const HIGGS_InputCandidate &, uint64_t &, Clock);
    HIGGS_HandStatus CopyInputRecognition(uint64_t, uint64_t, HIGGS_InputRecognition &);
    void NotifyInputObservations(const HIGGS_InputEvent *, uint32_t);
    void PrimeInputFrame(const HIGGS_ControllerFrame &);
    void AdvanceInputRecognitions(const HIGGS_ControllerFrame &);
    uint64_t InputSuppressionMask(uint32_t);
    void SetInputContext(uint32_t activeMenus, Clock);
    void ObserveContextFrame(const HIGGS_ControllerFrame &);
    bool GameplayInput(uint32_t hand, uint32_t button);
    bool GameplayContext();
    bool RebaseNativeInput(uint32_t hand);
    void ContextSuppression(uint32_t hand, uint64_t frame, uint64_t &pressed, uint64_t &touched);
    HIGGS_HandStatus CopyInputContext(uint64_t, HIGGS_InputContextState &);
    HIGGS_HandStatus CopyInputFrameContext(uint64_t, uint64_t, HIGGS_InputContextFrame &);
    HIGGS_HandStatus ContextEvents(uint64_t, HIGGS_InputContextCallback, void *, uint64_t &);
    using PhysicalDispatch = bool (*)(const HIGGS_PhysicalRequest &, void *);
    void DrainPhysical(PhysicalDispatch, void *, Clock);
    void SemanticEvent(uint32_t type, uint32_t hand, uint32_t ref, Clock);
    bool HasGrant(uint32_t hand, Clock);
    bool MayAcquire(uint32_t hand, Clock);
    bool MaySupport(uint32_t hand, Clock);
    HIGGS_HandStatus NativeRegionDetector(uint64_t, HIGGS_NativeRegionDetector, void*, uint64_t&);
    HIGGS_HandStatus RemoveNativeRegionDetector(uint64_t, uint64_t);
    HIGGS_HandStatus CopyNativeRegionFrame(uint64_t, uint32_t, HIGGS_NativeRegionFrame&);
    uint32_t EvaluateNativeRegions(HIGGS_NativeRegionFrame&, bool* usedDetector = nullptr);
    bool MayNativeRegionAction(uint32_t, uint64_t epoch, Clock);
    bool ConfigurePriorities(const std::string &settings);
    bool EffectivePriority(uint64_t, int32_t &);
    HIGGS_HandStatus HandUse(uint64_t, const HIGGS_HandUseRequest &, HIGGS_HandUseState &, uint64_t);
    HIGGS_HandStatus ReadHandUse(uint64_t, uint64_t, uint64_t, uint32_t, HIGGS_HandUseState &, uint64_t);
    HIGGS_HandStatus EndHandUse(uint64_t, uint64_t, uint64_t);
    HIGGS_HandStatus CopyHandUseTransition(uint64_t, uint64_t, HIGGS_HandUseTransitionState &, uint64_t);
    HIGGS_HandStatus ResizeHandUse(uint64_t, const HIGGS_HandUseTransitionRequest &, HIGGS_HandUseTransitionState &, uint64_t);
    void DrainHandUses();
    HIGGS_HandStatus ProposeHandUse(uint64_t, const HIGGS_HandUseProposalRequest &, uint64_t &, Clock);
    HIGGS_HandStatus CopyHandUseProposal(uint64_t, uint64_t, HIGGS_HandUseProposalState &, Clock);
    HIGGS_HandStatus CancelHandUseProposal(uint64_t, uint64_t, uint64_t);
    bool PhysicalPending(uint32_t hand, Clock);
    uint64_t Epoch() const;
    uint64_t HandGeneration(uint32_t hand) const;
    void ObserveSecondaryEpisode(uint32_t, uint64_t);
    uint64_t SecondaryEpisode(uint32_t) const;
    HIGGS_HandSnapshot CopyPublished() const;
    void ApplyDeferredWorld();
    bool MutationAdmissionReady(bool recovery = false);
    void MarkMainThread();
    HIGGS_HandStatus WithAuthorizedAction(uint64_t, const HIGGS_GrantedMutationRequest &, HIGGS_GrantedMutationCallback,
                                          void *, HIGGS_ActionResult &);
    HIGGS_HandStatus InvokeAuthorized(uint64_t, const HIGGS_AuthorizedDispatchRequest &,
                                      HIGGS_AuthorizedDispatchCallback, void *);
    std::unique_lock<std::recursive_mutex> NativePhase();
    void BeginHandMutation(Clock);
    void EndHandMutation(uint32_t hand, uint32_t heldRef, bool heldKnown, bool secondary, Clock);
    HIGGS_HandStatus Authorize(uint64_t owner, uint64_t executor, uint64_t recognizer, uint32_t hands);
    HIGGS_HandStatus Revoke(uint64_t owner, uint64_t executor, uint64_t recognizer);
    HIGGS_HandStatus CanRoute(uint64_t recognizer, uint64_t executor, uint32_t hands);
    HIGGS_HandStatus CancelRecognized(uint64_t recognizer, uint64_t request, uint64_t epoch, uint32_t reason);
    HIGGS_HandStatus Operation(uint64_t, const HIGGS_HandOperation &, uint64_t &, Clock);
    HIGGS_HandStatus EndOperation(uint64_t, uint64_t, uint64_t);
    HIGGS_HandStatus CopyOperation(uint64_t, uint64_t, HIGGS_HandOperationState &, Clock);
    using OperationDispatch = bool (*)(uint64_t, HIGGS_HandOperationState &, void *);
    void DrainOperations(OperationDispatch, void *, Clock);
    bool OperationActive(uint64_t, Clock);
    bool CopyActiveOperation(uint32_t hand, uint32_t kind, HIGGS_HandOperationState &, Clock);
    void ConfirmOperation(uint64_t, Clock);
    void BreakOperation(uint64_t, uint32_t);

  private:
    struct Client
    {
        uint64_t id = 0, label = 0;
        HIGGS_HandClientInfo info{};
        bool active = true;
        uint64_t drawEpoch = 0, drawUntil = 0;
        uint32_t drawDecision = HIGGS_DRAW_DEFAULT;
    };
    struct Registration
    {
        uint64_t id = 0, owner = 0;
        uint32_t kind = 0, inFlight = 0;
        bool active = true;
        HIGGS_RecognizerCallback recognizer = nullptr;
        HIGGS_ExecutorCallback executor = nullptr;
        HIGGS_HandEventCallback events = nullptr;
        HIGGS_TransferExecutorCallback transfer = nullptr;
        HIGGS_InputRecognizerSpec input{};
        HIGGS_InputContextCallback inputContext = nullptr;
        HIGGS_NativeRegionDetector nativeRegionDetector = nullptr;
        void *context = nullptr;
        std::vector<std::pair<uint64_t, uint32_t>> routes;
    };
    struct Action
    {
        HIGGS_ActionRequest request{};
        HIGGS_ActionResult result{};
        HIGGS_ActionCandidate candidate{};
        HIGGS_PhysicalRequest physical{};
        Clock expires = 0;
        bool grantActive = false, selected = false, dispatched = false;
        uint64_t recognizerOwner = 0;
        uint64_t stableTarget = 0;
        HIGGS_HandOperationState operation{};
        bool operationActive = false;
        bool nativeStarted = false;
        bool authorizedDispatchClaimed = false;
        bool retained = false;
        bool domainOwned = false;
        uint32_t ownedActuator = 0;
        HIGGS_InputCandidate inputSpec{};
        HIGGS_InputRecognition recognition{};
        uint32_t inputControl = UINT32_MAX;
        bool inputPressed = false, inputTouch = false;
        uint64_t inputDownAt = 0, inputLastReleaseAt = 0;
        uint64_t inputExecutionExpires = 0, inputHandGenerations[2]{};
        uint64_t inputRegistration = 0;
        uint64_t inputContextGeneration = 0;
        uint32_t sessionKind = 0;
        uint64_t sessionRevision = 0;
        uint64_t dependencyGrant = 0;
        uint32_t dependencyHands = 0;
        bool sessionNotified = false;
        HIGGS_HandUseProposalRequest useProposal{};
        uint64_t proposalRound = 0;
        int32_t proposalPriority = 0;
        uint32_t proposalDecisionFlags = 0;
    };
    HIGGS_HandStatus Add(HIGGS_HandClient, uint32_t, HIGGS_RecognizerCallback, HIGGS_ExecutorCallback,
                         HIGGS_HandEventCallback, void *, uint64_t &);
    Client *ClientLocked(uint64_t);
    std::string_view ClientKeyLocked(uint64_t);
    std::shared_ptr<Registration> RegistrationLocked(uint64_t);
    Action *ActionLocked(uint64_t);
    Action *NewActionLocked();
    void ExpireLocked(Clock);
    void ResolveHandUseProposalsLocked(Clock);
    bool EligibleLocked(uint32_t hands) const;
    bool GrantReadyLocked(uint64_t owner, uint64_t grant) const;
    bool InputEligibleLocked(uint32_t hands, uint64_t executorOwner, uint64_t parentGrant,
                             bool maintainingRecognition = false) const;
    bool GameplayInputLocked(uint32_t hand, uint32_t button) const;
    HIGGS_InputContextState ContextStateLocked() const;
    bool RoutedLocked(uint64_t executor, uint64_t recognizer, uint32_t hands);
    int32_t PriorityLocked(const Client &, bool &overridden) const;
    int32_t ActionPriorityLocked(const Client &, uint64_t action, bool &overridden) const;
    uint64_t GrantCorrelationLocked(uint64_t grant) const;
    uint64_t ClientLabelLocked(uint64_t);
    void ObserveRecordLocked(HIGGS_HandRecord &);
    HIGGS_HandSnapshot ObservationLocked(HIGGS_HandSnapshot);
    void EndSequenceLocked(size_t hand, size_t control, uint64_t up, uint32_t reason);
    void CancelSequencesLocked(size_t hand, uint32_t reason);
    void CancelRoutesLocked(uint64_t executor, uint64_t owner, uint64_t recognizer, uint32_t reason);
    void RefreshLocked();
    void FinishLocked(Action &, uint32_t stage, uint32_t reason);
    void QueueResultLocked(const Action &);
    void QueueEventLocked(uint32_t, uint32_t, uint32_t, Clock, const HIGGS_HandRecord *);
    void FlushEvents();
    bool Enter(const std::shared_ptr<Registration> &);
    void InvokeSessionExecutor(const std::shared_ptr<Registration> &, const HIGGS_ActionRequest &);
    void Leave(const std::shared_ptr<Registration> &);
    std::vector<std::shared_ptr<Registration>> List(uint32_t);
    mutable std::mutex mutex_;
    std::condition_variable drained_;
    std::recursive_mutex dispatch_;
    std::vector<Client> clients_;
    uint64_t drawBoundaryOwner_ = 0, drawBoundaryId_ = 0;
    uint64_t nextClientLabel_ = UINT64_C(1) << 63;
    std::vector<std::pair<std::string, int32_t>> priorities_;
    std::vector<std::shared_ptr<Registration>> registrations_;
    std::array<Action, 256> actions_{};
    std::array<HIGGS_InputDecision, 256> decisions_{};
    std::array<uint64_t, 256> decisionContexts_{};
    struct ActiveSequence {
        HIGGS_InputSequence record{};
        uint64_t owner = 0, recognizer = 0;
    };
    std::vector<std::array<ActiveSequence,kInputControlSlots>> activeSequences_{2};
    std::vector<HIGGS_InputSequence> pastSequences_ = std::vector<HIGGS_InputSequence>(256);
    std::array<std::array<uint64_t,2>,2> nativeFallbackPress_{};
    size_t nextPastSequence_ = 0;
    std::vector<std::pair<uint64_t,HIGGS_HandEvent>> pendingEvents_;
    std::vector<std::pair<HIGGS_InputContextState, HIGGS_InputContextState>> pendingContexts_;
    uint32_t contextMenus_ = 0;
    uint64_t contextGeneration_ = 1, contextChangedAt_ = 0;
    struct ContextHand {
        uint64_t frame = 0, pressed = 0, touched = 0, drainPressed = 0, drainTouched = 0;
        uint32_t device = UINT32_MAX;
        uint64_t nativeGeneration = 0;
        bool known = false, touchGrip = false;
    } contextHands_[2];
    std::array<HIGGS_InputContextFrame, 128> contextFrames_{};
    HIGGS_HandSnapshot snapshot_{};
    uint32_t nativeUnavailableReasons_[2]{};
    uint64_t secondaryEpisodes_[2]{};
    uintptr_t world_ = 0;
    bool loading_ = false;
    bool worldChangePending_ = false, forceWorldReset_ = false, pendingWorldLoading_ = false;
    uintptr_t pendingWorld_ = 0;
    Clock pendingWorldAt_ = 0;
    std::thread::id mainThread_{};
    bool down_[2][kInputControlSlots]{}, consumed_[2][kInputControlSlots]{};
    uint64_t nextToken_ = 1, nextInput_ = 1, nextObservation_ = 1, nextEvent_ = 1;
    uint64_t nextUseRound_ = 1;
    bool drainingHandUses_ = false; // serialized by dispatch_, including callback reentry
    HIGGS_NativeRegionFrame nativeRegionFrames_[2]{};
    uint64_t nativeRegionContexts_[2]{};
    uint64_t nextNativeRegionSample_ = 1;
    bool nativeRegionCallback_ = false; // serialized by dispatch_, rejects recursive detector calls
};
Broker &Service();
Clock Now();
void ObserveWorld(uintptr_t, bool loading = false);
void BeginLoad();
void EndLoad();
void NewGame();
bool Input(uint32_t hand, uint32_t button, bool down, bool &nativeContinuation);
bool HasGrant(bool isLeft);
bool MayAcquire(bool isLeft);
bool MaySupport(bool isLeft);
// Native adapter publishes after PostVrikUpdate and before public post callbacks.
void PublishNative();
void DrainNative();
void FinishNative();
bool CopyNativeTarget(bool isLeft, float position[3], float rotation[9], float &scale);
bool HasNativeTarget(bool isLeft);
void ConfirmNativeTarget(bool isLeft);
void ApplyDriveCap(bool isLeft, float &linear, float &angular);
bool CopyHandGeometry(bool isLeft, float halfExtents[3]);
void ConfirmNativeGrip(bool isLeft);
} // namespace HiggsHands
