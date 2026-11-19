#include "higgsphysics.h"
_Static_assert(sizeof(HIGGS_PhysicalPolicyRequest) == 152, "policy request ABI");
_Static_assert(sizeof(HIGGS_PhysicalPolicyValues) == 52, "policy values ABI");
_Static_assert(sizeof(HIGGS_PhysicalPolicyState) == 336, "policy state ABI");
_Static_assert(sizeof(HIGGS_PhysicalPolicyInterface) == 48, "policy table ABI");
int policy_c(void)
{
    HIGGS_PhysicalPolicyInterface api = {0};
    return HIGGS_QueryPhysicalPolicyInterface(2, sizeof(api), &api) || !api.End;
}
