#pragma once
#include "controller_tracking.h"
#include "higgsobservations.h"

namespace HiggsMotion {
    // Pure projection of one atomic owner read; no clock, engine or state mutation.
    HIGGS_MotionSnapshot VelocitySnapshot(const CopiedTracking&);
}
