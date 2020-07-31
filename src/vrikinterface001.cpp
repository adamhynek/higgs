#include "vrikinterface001.h"

// A message used to fetch VRIK's interface
struct VrikMessage {
	enum { kMessage_GetInterface = 0xF2AFAEE6 }; // Randomly chosen by cat
	void * (*getApiFunction)(unsigned int revisionNumber) = nullptr;
};

// Fetches the interface to use from VRIK
vrikPluginApi::IVrikInterface001 * vrikPluginApi::getVrikInterface001(const PluginHandle & pluginHandle, SKSEMessagingInterface * messagingInterface)
{
	// If the interface has already been fetched, rturn the same object
	if (g_vrikInterface) {
		return g_vrikInterface;
	}

	// Dispatch a message to get the plugin interface from VRIK
	VrikMessage vrikMessage;
	messagingInterface->Dispatch(pluginHandle, VrikMessage::kMessage_GetInterface, (void*)&vrikMessage, sizeof(VrikMessage*), "VRIK");
	if (!vrikMessage.getApiFunction) {
		return nullptr;
	}

	// Fetch the API for this version of the VRIK interface
	g_vrikInterface = static_cast<IVrikInterface001*>(vrikMessage.getApiFunction(1));
	return g_vrikInterface;
}
