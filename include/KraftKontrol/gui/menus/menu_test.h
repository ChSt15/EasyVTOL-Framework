#ifndef MENU_TEST_H
#define MENU_TEST_H



#include "../menu_abstract.h"

#include "KraftKontrol/utils/topic_subscribers.h"



/**
 * Simple menu to test if everything is working
 */
class Menu_Test: public Menu_Abstract {
private:

    //Control for menu
    Buffer_Subscriber<eMenuControlInput_t, 10> menuControlSubscriber_;


public:

    /**
     * @param menuControl Which device to use as control for menu.
     */
    Menu_Test(const char* menuListName, MenuControl_Abstract& menuControl);


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


};



#endif