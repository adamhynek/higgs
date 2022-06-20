# HIGGS

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

[Nexus link](https://www.nexusmods.com/skyrimspecialedition/mods/43930)
