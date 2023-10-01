#pragma once
#include "skse64/PluginAPI.h"

namespace vrikPluginApi {

    // Returns an IVrikInterface001 object compatible with the API shown below
    // This should only be called after SKSE sends kMessage_PostLoad to your plugin
    class IVrikInterface001;
    IVrikInterface001 * getVrikInterface001(const PluginHandle & pluginHandle, SKSEMessagingInterface * messagingInterface);

    // This object provides access to VRIK's mod support API
    class IVrikInterface001
    {
    public:
        // Gets the VRIK build number
        virtual unsigned int getBuildNumber() = 0;

        // Getters and setters for INI keys and mod settings (files are not saved automatically)
        virtual double getSettingDouble(const char * name) = 0;
        virtual void   setSettingDouble(const char * name, double value) = 0;
        virtual void   getSettingString(const char * name, char * buffer, size_t bufferSize) = 0;
        virtual void   setSettingString(const char * name, const char * value) = 0;

        // Save changes to disk, or restore all non-numeric (doubles) to their saved values
        virtual void   saveSettings() = 0;
        virtual void   restoreSettings() = 0;

        // Create gesture mod actions and profiles from SKSE plugins
        typedef void(*GestureCallback)(int pressCount);
        virtual void addGestureAction(GestureCallback callback, const char * mcmMenuName) = 0;
        virtual void beginGestureProfile() = 0;
        virtual void setProfileAction(int gestureNumber, GestureCallback callback) = 0;
        virtual void endGestureProfile() = 0;
        // Fetch VRIK finger position (0=closed to 1=open), and set min/max finger positions
        // VRIK normally controls the range of fingers on its own.  Return control to VRIK with restoreFingers when done.
        // Fingers are indexed as follows: 0=thumb, 1=index, 2=middle, 3=ring, 4=pinky
        virtual float getFingerPos(bool isLeft, int fingerIndex) = 0;
        virtual void  setFingerRange(bool isLeft, float min0, float max0, float min1, float max1, float min2, float max2, float min3, float max3, float min4, float max4) = 0;
        virtual void  restoreFingers(bool isLeft) = 0;
    };
}

extern vrikPluginApi::IVrikInterface001 * g_vrikInterface;
