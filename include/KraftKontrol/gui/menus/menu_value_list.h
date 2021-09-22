#ifndef MENU_VALUE_LIST_H
#define MENU_VALUE_LIST_H



#include "../menu_abstract.h"

#include "KraftKontrol/utils/topic_subscribers.h"



/**
 * This menu shows on the display the value of a given item. The name of the menu will also display the value.
 */
class Menu_ValueList: public Menu_Abstract {
private:

    struct MenuValue {
        /* data */
    };
    

    //List of menus to show
    const char* (*valueFunc)(void) = nullptr;

    //Control for menu
    Simple_Subscriber<eMenuControlInput_t> menuControlSubscriber_;

    MenuControl_Abstract& menuControl_;


public:

    /**
     * @param menuValueName Title or identifier of value to be shown.
     * @param valueFunc Function to call the gets the value and returns a char pointer to a formatted value.
     * @param menuControl Control input for menu. 
     */
    Menu_ValueList(const char* menuValueName, const char* (*valueFunc)(void), MenuControl_Abstract& menuControl);


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
    Menu_Abstract* menuDisplayUpdate(Display_Abstract& display, Task_Abstract& gui) override;


    void exit() override;

    void enter() override;


};



#endif