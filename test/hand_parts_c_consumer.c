#include "higgsphysics.h"
#include "higgsnativeinterop.h"
#include <stddef.h>
_Static_assert(sizeof(HIGGS_HandPartSources)==1104,"sources");
_Static_assert(sizeof(HIGGS_HandPartDefinition)==76,"definition");
_Static_assert(sizeof(HIGGS_HandPartSetRequest)==664,"request");
_Static_assert(sizeof(HIGGS_HandPartBody)==120,"body");
_Static_assert(sizeof(HIGGS_HandPartSetState)==1056,"state");
_Static_assert(sizeof(HIGGS_HandPartAttribution)==56,"attribution");
_Static_assert(sizeof(HIGGS_HandPartContact)==72,"contact");
_Static_assert(sizeof(HIGGS_HandPartsInterface)==64,"table");
_Static_assert(sizeof(HIGGS_HandPartsNativeInterface)==24,"native table");
int HandPartsCLayout(void){return (int)offsetof(HIGGS_HandPartSetRequest,parts);}
