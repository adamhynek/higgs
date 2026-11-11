#pragma once
#include <stdint.h>

namespace HiggsGrip {
    uint64_t Epoch();
    void Invalidate();
    void SetLoading(bool loading);
    bool Select(bool isLeft, uint64_t epoch);
}
