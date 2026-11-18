#include "higgsinput.h"
#include "higgsinput.h"
#include <stddef.h>
_Static_assert(sizeof(HIGGS_ControllerFrame)==144,"C input frame layout");
_Static_assert(sizeof(HIGGS_InputInterface)==64,"C extended input table layout");
_Static_assert(sizeof(HIGGS_InputContextState)==88,"C context state layout");
_Static_assert(sizeof(HIGGS_InputContextFrame)==56,"C context frame layout");
_Static_assert(sizeof(HIGGS_InputContextInterface)==40,"C context table layout");
_Static_assert(HIGGS_CONTROL_MENU==1&&HIGGS_CONTROL_A==7&&HIGGS_CONTROL_TRIGGER==33,"fixed control IDs");
unsigned InputCLayout(void){return (unsigned)offsetof(HIGGS_ControllerFrame,axes);}
