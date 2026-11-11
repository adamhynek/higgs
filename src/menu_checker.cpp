#include "menu_checker.h"
#include "hand_broker.h"
#include <mutex>

// Big thanks to Shizof for this method of checking what menus are open
namespace MenuChecker
{
    namespace { std::mutex menuMutex; }
    std::vector<std::string> iniChangeMenus{
        "Console",
        "Journal Menu"
    };
    
    std::vector<std::string> gameStoppingMenus{
        "BarterMenu",
        "Book Menu",
        "Console",
        "Native UI Menu",
        "ContainerMenu",
        "Dialogue Menu",
        "Crafting Menu",
        "CustomMenu",
        "Credits Menu",
        "Debug Text Menu",
        "FavoritesMenu",
        "GiftMenu",
        "InventoryMenu",
        "Journal Menu",
        "Kinect Menu",
        "Loading Menu",
        "Lockpicking Menu",
        "MagicMenu",
        "Main Menu",
        "MapMarkerText3D",
        "MapMenu",
        "MessageBoxMenu",
        "Mist Menu",
        "Quantity Menu",
        "RaceSex Menu",
        "Sleep/Wait Menu",
        "StatsMenuSkillRing",
        "StatsMenuPerks",
        "Training Menu",
        "Tutorial Menu",
        "TweenMenu"
    };

    std::unordered_map<std::string, bool> menuTypes({
        { "BarterMenu", false },
        { "Book Menu", false },
        { "Console", false },
        { "Native UI Menu", false },
        { "ContainerMenu", false },
        { "Dialogue Menu", false },
        { "Crafting Menu", false },
        { "CustomMenu", false },
        { "Credits Menu", false },
        { "Cursor Menu", false },
        { "Debug Text Menu", false },
        { "Fader Menu", false },
        { "FavoritesMenu", false },
        { "GiftMenu", false },
        { "HUD Menu", false },
        { "InventoryMenu", false },
        { "Journal Menu", false },
        { "Kinect Menu", false },
        { "Loading Menu", false },
        { "Lockpicking Menu", false },
        { "MagicMenu", false },
        { "Main Menu", false },
        { "MapMarkerText3D", false },
        { "MapMenu", false },
        { "MessageBoxMenu", false },
        { "Mist Menu", false },
        { "Overlay Interaction Menu", false },
        { "Overlay Menu", false },
        { "Quantity Menu", false },
        { "RaceSex Menu", false },
        { "Sleep/Wait Menu", false },
        { "StatsMenu", false },
        { "StatsMenuPerks", false },
        { "StatsMenuSkillRing", false },
        { "TitleSequence Menu", false },
        { "Top Menu", false },
        { "Training Menu", false },
        { "Tutorial Menu", false },
        { "TweenMenu", false },
        { "WSEnemyMeters", false },
        { "WSDebugOverlay", false },
        { "WSActivateRollover", false },
        { "LoadWaitSpinner", false }
    });

    // Menu open event functions
    AllMenuEventHandler menuEvent;
    
    EventResult AllMenuEventHandler::ReceiveEvent(MenuOpenCloseEvent * evn, EventDispatcher<MenuOpenCloseEvent> * dispatcher)
    {
        if (!evn || !evn->menuName.data) return EventResult::kEvent_Continue;
        // Match the controller/native serialization order. Never call broker code
        // while holding menuMutex: native publication reads this menu set too.
        auto phase = HiggsHands::Service().NativePhase();
        const char * menuName = evn->menuName.data;
        uint32_t active = 0;
        {
        std::lock_guard lock(menuMutex);
                
        if (evn->opening) { // Menu opened
            if (menuTypes.find(menuName) != menuTypes.end()) {
                menuTypes[menuName] = true;
            }
        }
        else { // Menu closed
            if (menuTypes.find(menuName) != menuTypes.end()) {
                menuTypes[menuName] = false;
            }			
        }
        for (const auto &name : gameStoppingMenus)
            if (menuTypes[name]) ++active;
        }
        HiggsHands::Service().SetInputContext(active, HiggsHands::Now());
        return EventResult::kEvent_Continue;
    }

    bool isGameStopped()
    {
        std::lock_guard lock(menuMutex);
        for (const auto &name : gameStoppingMenus) {
            if (menuTypes[name]) {
                return true;
            }
        }
        return false;
    }
}
