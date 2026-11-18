#include "higgsnativeinterop.h"
#include "higgsphysics.h"
#include <stddef.h>
_Static_assert(sizeof(HIGGS_ContactItem)==64,"contact identity layout");
_Static_assert(sizeof(HIGGS_CosmeticContactPolicy)==80,"policy layout");
_Static_assert(sizeof(HIGGS_CosmeticContactState)==56,"lease layout");
_Static_assert(sizeof(HIGGS_CosmeticContact)==272,"evidence layout");
_Static_assert(sizeof(HIGGS_ContactItemFrame)==304,"source frame layout");
_Static_assert(sizeof(HIGGS_NativeOwnContact)==104,"native scope layout");
_Static_assert(sizeof(HIGGS_CosmeticContactInterface)==56,"copied table layout");
_Static_assert(sizeof(HIGGS_OwnContactNativeInterface)==24,"native table layout");
int own_contact_c_probe(void){HIGGS_CosmeticContactInterface api={0};if(HIGGS_QueryCosmeticContactInterface(99,sizeof(api),&api)!=HIGGS_HAND_UNSUPPORTED_VERSION)return 1;return HIGGS_QueryCosmeticContactInterface(1,sizeof(api),&api)!=HIGGS_HAND_OK;}
