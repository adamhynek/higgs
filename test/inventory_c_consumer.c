#include "higgstransactions.h"
#include <stddef.h>
_Static_assert(sizeof(HIGGS_InventorySelectionRequest)==32,"selection request ABI");
_Static_assert(sizeof(HIGGS_InventorySelection)==104,"selection ABI");
_Static_assert(offsetof(HIGGS_InventorySelection,item)==32,"item offset");
_Static_assert(sizeof(HIGGS_InventoryInterface)==48,"table ABI");
int inventory_c_probe(void){
    HIGGS_InventoryInterface table={0};
    if(HIGGS_QueryInventoryInterface(999,sizeof(table),&table)!=HIGGS_HAND_UNSUPPORTED_VERSION)return 1;
    if(HIGGS_QueryInventoryInterface(2,sizeof(table)-1,&table)!=HIGGS_HAND_BUFFER_TOO_SMALL)return 2;
    return HIGGS_QueryInventoryInterface(2,sizeof(table),&table)!=HIGGS_HAND_OK;
}
