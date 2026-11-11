#include "higgsgripactions.h"
#include "config.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <cstring>
#include <mutex>

namespace {
struct Entry {
    uint64_t token = 0;
    int priority = 0;
    char id[64]{};
    HiggsGripOffer offer = nullptr;
    HiggsGripSelected selected = nullptr;
    void* context = nullptr;
    unsigned inFlight = 0;
    bool active = false;
};
std::array<Entry, 32> entries;
std::mutex gate;
uint64_t nextToken = 1;
std::atomic<uint64_t> contextGeneration{1};
std::atomic<bool> loading{false};

uint32_t Register(const char* id, HiggsGripOffer offer, HiggsGripSelected selected, void* context, uint64_t* token)
{
    if (!id || !offer || !selected || !token) return HIGGS_GRIP_INVALID;
    const size_t length = strnlen(id, sizeof(Entry::id));
    if (!length || length == sizeof(Entry::id) || std::strspn(id, "abcdefghijklmnopqrstuvwxyz0123456789._-") != length)
        return HIGGS_GRIP_INVALID;

    std::lock_guard lock(gate);
    for (const auto& entry : entries)
        if (entry.token && std::strcmp(entry.id, id) == 0) return HIGGS_GRIP_CONFLICT;
    auto free = std::find_if(entries.begin(), entries.end(), [](const Entry& entry) { return !entry.token; });
    if (free == entries.end() || !nextToken) return HIGGS_GRIP_FULL;

    // Read once at registration: priorities cannot change in the middle of a press.
    int priority = 0;
    try {
        const auto setting = Config::GetConfigOption("GripActions", id);
        if (!setting.empty()) {
            const auto value = std::from_chars(setting.data(), setting.data() + setting.size(), priority);
            if (value.ec != std::errc{} || value.ptr != setting.data() + setting.size()) priority = 0;
        }
    } catch (...) { return HIGGS_GRIP_INVALID; }
    *free = {nextToken++, priority};
    std::memcpy(free->id, id, length + 1);
    free->offer = offer;
    free->selected = selected;
    free->context = context;
    free->active = true;
    *token = free->token;
    return HIGGS_GRIP_OK;
}

uint32_t Remove(uint64_t token)
{
    if (!token) return HIGGS_GRIP_INVALID;
    std::lock_guard lock(gate);
    for (auto& entry : entries) {
        if (entry.token != token) continue;
        entry.active = false;
        if (entry.inFlight) return HIGGS_GRIP_BUSY;
        entry = {};
        return HIGGS_GRIP_OK;
    }
    return HIGGS_GRIP_INVALID;
}
}

extern "C" const HiggsGripActions001* HIGGS_GetGripActions001()
{
    static const HiggsGripActions001 api{Register, Remove};
    return &api;
}

uint64_t HiggsGrip::Epoch()
{
    return loading.load() ? 0 : contextGeneration.load();
}

void HiggsGrip::Invalidate()
{
    ++contextGeneration;
}

void HiggsGrip::SetLoading(bool value)
{
    if (value) loading = true;
    Invalidate();
    if (!value) loading = false;
}

bool HiggsGrip::Select(bool isLeft, uint64_t epoch)
{
    struct Candidate { Entry* source; Entry copy; };
    std::array<Candidate, 32> candidates;
    size_t count = 0;
    {
        std::lock_guard lock(gate);
        for (auto& entry : entries)
            if (entry.active && entry.priority >= 0) candidates[count++] = {&entry, entry};
    }
    std::sort(candidates.begin(), candidates.begin() + count, [](const Candidate& a, const Candidate& b) {
        if (a.copy.priority != b.copy.priority) return a.copy.priority > b.copy.priority;
        return std::strcmp(a.copy.id, b.copy.id) < 0;
    });
    for (size_t i = 0; i < count; ++i) {
        auto& candidate = candidates[i];
        auto& entry = *candidate.source;
        {
            std::lock_guard lock(gate);
            if (Epoch() != epoch) return false;
            if (entry.token != candidate.copy.token || !entry.active) continue;
            ++entry.inFlight;
        }
        uint64_t action = 0;
        try { action = candidate.copy.offer(isLeft, candidate.copy.context); } catch (...) {}
        {
            std::lock_guard lock(gate);
            if (!action || !entry.active || Epoch() != epoch) {
                --entry.inFlight;
                continue;
            }
        }
        // A callback that already produced a side effect must never trigger a second choice.
        try { candidate.copy.selected(isLeft, action, candidate.copy.context); } catch (...) {}
        std::lock_guard lock(gate);
        --entry.inFlight;
        return true;
    }
    return false;
}
