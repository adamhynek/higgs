#include "higgsobservations.h"
#include <stddef.h>
_Static_assert(sizeof(HIGGS_MotionSample)==184,"motion C layout");
_Static_assert(sizeof(HIGGS_TrackingMappingSample)==112,"mapping C layout");
_Static_assert(sizeof(HIGGS_ContactObservation)==128,"contact C layout");
_Static_assert(sizeof(HIGGS_ObservationAPI)==48,"table C layout");
unsigned ObservationCLayout(void){return (unsigned)offsetof(HIGGS_MotionSample,position);}
