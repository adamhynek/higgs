# HIGGS ([Nexus link](https://www.nexusmods.com/skyrimspecialedition/mods/43930))

This mod is complicated.

To build from source, it requires object definitions / headers similar to those belonging to havok 2010.2. I will not be providing these, nor do I claim to have them myself.


HIGGS has a C++ API that can be used by other mods.\
To use it, copy `src/higgsinterface001.cpp` and `include/higgsinterface001.h` into your project.\
Then, do something like this in your SKSE plugin in `PostPostLoad` or later (this is important - if you try and get the interface before `PostPostLoad`, such as in `PostLoad`, it will not work).

```cpp
#include "higgsinterface001.h"

...

void OnSKSEMessage(SKSEMessagingInterface::Message* msg)
{
  if (msg) {
    if (msg->type == SKSEMessagingInterface::kMessage_PostPostLoad) {
      // Get the HIGGS plugin API
      HiggsPluginAPI::GetHiggsInterface001(g_pluginHandle, g_messaging);
      if (g_higgsInterface) {
        _MESSAGE("Got higgs interface!");
        unsigned int higgsVersion = g_higgsInterface->GetBuildNumber();
      }
    }
  }
}
        
```

## Optional one-shot grip actions

A pouch mod can offer an action when an idle player's physical hand presses grip. HIGGS selects one eligible provider before its own grab starts. Without an offer the original input path runs. A winning action consumes the grip through release. Trigger input, an existing grab, and menus keep their usual handling.

After SKSE PostPostLoad, look up `HIGGS_GetGripActions001` in the loaded HIGGS DLL and use [its C header](include/higgsgripactions.h). The offer callback reads local context and returns an action ID. Only the selected callback queues that action. Revalidate the target on the game thread before performing deferred work.

Players can set integer scores in the `[GripActions]` section of `higgs_vr.ini`, for example `example.pouch=20`. Higher scores are tried first. Equal scores use the stable provider ID in ascending ASCII order. Missing or invalid scores default to zero. A negative score disables that provider. Settings apply on registration, normally at game startup.

This first proposal covers individual grip presses. Continuous hand ownership and arbitrary physical operations are separate proposals in the [synergy concept](https://github.com/Vhodnoylogin/body-pouches/blob/main/docs/trinity-synergy/concept-en.md).
