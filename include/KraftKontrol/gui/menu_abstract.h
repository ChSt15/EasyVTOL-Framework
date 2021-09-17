#ifndef MENU_ABSTRACT_H
#define MENU_ABSTRACT_H



#include "KraftKontrol/modules/hid_modules/display_modules/display_abstract.h"

#include "KraftKontrol/utils/Simple-Schedule/task_autorun_class.h"

#include "KraftKontrol/utils/topic.h"

#include "string.h"



class Gui;



///Maximum  length of menu item name.
constexpr uint16_t MENU_MAX_NAME_SIZE = 40;


/**
 * Event enum for transfering user input data to menu.
 * Short means signal was a single press.
 * Long means signal is being held down.
 * Can be continued by creating an enum starting at eMenuControlInput_END
 */
enum eMenuControlInput_t: uint32_t {
    //These functions must be supported.
    eMenuControlInput_UpShort,
    eMenuControlInput_DownShort,
    eMenuControlInput_LeftShort,
    eMenuControlInput_RightShort,
    eMenuControlInput_UpLong,
    eMenuControlInput_DownLong,
    eMenuControlInput_LeftLong,
    eMenuControlInput_RightLong,
    //These functions are not needed
    eMenuControlInput_Button1Short,
    eMenuControlInput_Button2Short,
    eMenuControlInput_Button3Short,
    eMenuControlInput_Button4Short,
    eMenuControlInput_Button5Short,
    eMenuControlInput_Button6Short,
    eMenuControlInput_Button7Short,
    eMenuControlInput_Button8Short,
    eMenuControlInput_Button9Short,
    eMenuControlInput_Button10Short,
    eMenuControlInput_Button1Long,
    eMenuControlInput_Button2Long,
    eMenuControlInput_Button3Long,
    eMenuControlInput_Button4Long,
    eMenuControlInput_Button5Long,
    eMenuControlInput_Button6Long,
    eMenuControlInput_Button7Long,
    eMenuControlInput_Button8Long,
    eMenuControlInput_Button9Long,
    eMenuControlInput_Button10Long,
    //Use this to continue enum if needed.
    eMenuControlInput_END
};



/**
 * This allows buttons, encoders, joysticks etc. to be used to control menus.
 */
class MenuControl_Abstract {
protected:

    Topic<eMenuControlInput_t> menuInputTopic_;


public:

    Topic<eMenuControlInput_t>& getMenuInputTopic() {return menuInputTopic_;}


};



/**
 * All menus should inheret from this class to support being placed into GUIs.
 * Menus control what to do for user inputs and can choose which next menu to display or to return to previous display.
 * Every menu must implement menuDisplayUpdate() function and the return pointer controls which menu to display.
 */
class Menu_Abstract {
friend Gui;
private:

    ///Char array containing menu item name for display.
    char menuName_[MENU_MAX_NAME_SIZE];


protected:

    /**
     * Constructor for Menu abstract.
     * @param menuName Char array containing name of menu with max length of MENU_MAX_NAME_SIZE.
     */
    Menu_Abstract(const char* menuName);

    //Make sure menu cant be deleted from outside. Must stay private.
    ~Menu_Abstract() {}

    /**
     * Changes the menu name to given name.
     * @param menuName Char array containing new name of menu.
     */
    void setMenuName(const char* menuName);


public:

    /**
     * @returns a pointer to the menus char array containing the name of the menu.
     */
    const char* getMenuName();


protected:

    /**
     * Virtual method to display current menu status for the GUI.
     * Subclass must implement this function and write to the given display.
     * 
     * Menu return function works as following:
     * - returning a pointer to this menu means to stay on this menu.
     * - returning a nullptr means to return to last menu.
     * - returning a pointer to another menu will switch to that menu. 
     * 
     * @param display Display to be updated with menu interface.
     * @param gui Gui task that called this menu. Can be used to change display fps, etc. 
     * @returns pointer to Menu_Abstract.
     */
    virtual Menu_Abstract* menuDisplayUpdate(Display_Abstract& display, Task_Abstract& gui) = 0;

    /**
     * Will be called once before menuDisplayUpdate() is called when menu is selected. 
     * To be implemented by subclasses. Places display into known mode.
     */
    virtual void resetMenu() {}


};



#endif