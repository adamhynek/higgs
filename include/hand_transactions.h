#pragma once
#include "higgstransactions.h"
#include <array>
#include <mutex>
#include <vector>
namespace HiggsHands
{
class Broker;
class TransactionService
{
  public:
    explicit TransactionService(Broker &broker) : broker_(broker)
    {
    }
    HIGGS_HandStatus Declare(uint64_t, const HIGGS_ItemDeclaration &, HIGGS_ItemIdentity &);
    HIGGS_HandStatus Held(uint64_t, uint32_t, HIGGS_ItemIdentity &);
    HIGGS_HandStatus Request(uint64_t, const HIGGS_TransferRequest &, uint64_t &, uint64_t);
    HIGGS_HandStatus Begin(uint64_t, uint64_t, uint64_t, uint64_t &);
    HIGGS_HandStatus Report(uint64_t, const HIGGS_TransferReceipt &);
    HIGGS_HandStatus Copy(uint64_t, uint64_t, HIGGS_TransferState &);
    HIGGS_HandStatus Cancel(uint64_t, uint64_t, uint64_t);
    void Drain(uint64_t);
    uint64_t ObserveHeld(uint32_t, uint64_t, uint32_t, uint32_t, uint32_t, uint64_t);
    HIGGS_HandStatus ObserveInventory(uint64_t owner, uint64_t ticketKey, uint64_t version, uint32_t baseForm,
                                      uint32_t quantity, HIGGS_ItemIdentity &);
    void InvalidateInventory(uint64_t owner, uint64_t ticketKey);
    bool RetireInventory(uint64_t owner, uint64_t ticketKey);
    bool RetireNativeItem(uint64_t instance);
    // Internal owner query: client unregistration revokes authority, not an
    // already issued mutation's native lifetime or unresolved recovery pin.
    bool NativePinRequired(uint64_t owner, uint64_t request, uint64_t epoch, HIGGS_TransferState &);
    bool ItemInUse(uint64_t);
    bool IdentityCurrent(const HIGGS_ItemIdentity &);

  private:
    struct Item
    {
        HIGGS_ItemIdentity value{};
        uint64_t nativeKey = 0, seen = 0;
        bool available = true;
    };
    struct Transaction
    {
        HIGGS_TransferRequest request{};
        HIGGS_TransferState state{};
        HIGGS_ActionRequest authority{};
        HIGGS_TransferReceipt lastReceipt{};
        HIGGS_TransferReceipt compensationReceipt{};
        uint64_t deadline = 0;
        uint64_t equipmentObservation = 0;
        bool ownGrant = false, notified = false, permitIssued = false, compensationPermit = false;
    };
    void SyncLocked();
    Transaction *Find(uint64_t);
    Item *FindItem(uint64_t);
    bool IdentityLocked(const HIGGS_ItemIdentity &) const;
    bool LockedItem(uint64_t) const;
    void Finish(Transaction &, uint32_t, uint32_t, uint32_t);
    void NeedCompensation(Transaction &, uint32_t, uint64_t);
    bool Alive(Transaction &, uint64_t);
    Broker &broker_;
    std::mutex mutex_;
    std::array<Item, 256> items_{};
    std::array<Transaction, 128> transactions_{};
    struct Key
    {
        uint64_t owner, epoch, request;
        HIGGS_TransferRequest input;
    };
    std::vector<Key> keys_;
    uint64_t epoch_ = 0, next_ = 1, held_[2]{};
};
TransactionService &Transactions();
void PublishNativeItems();
} // namespace HiggsHands
