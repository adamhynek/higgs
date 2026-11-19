#pragma once
#include "higgsabitypes.h"
#include <cstring>

namespace HiggsAbi {
template<class Operation>
HIGGS_HandStatus Guard(Operation operation) noexcept {
    try {
        return operation();
    } catch (...) {
        return HIGGS_HAND_INTERNAL_ERROR;
    }
}

// All structural input and output-capacity checks precede entry here.
// A nonzero status or exception preserves every byte of the caller's output.
template<class Value, class Operation>
HIGGS_HandStatus PublishOnSuccess(Value* out, Operation operation) noexcept {
    return Guard([&] {
        Value result{};
        const auto status = operation(result);
        if (status == HIGGS_HAND_OK)
            std::memcpy(out, &result, sizeof(result));
        return status;
    });
}

template<class Record>
HIGGS_HandStatus CheckSized(Record* out) noexcept {
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (out->structSize < sizeof(Record)) {
        out->structSize = sizeof(Record);
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    }
    return HIGGS_HAND_OK;
}
}
