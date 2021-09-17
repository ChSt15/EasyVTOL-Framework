#ifndef MENU_LIST_H
#define MENU_LIST_H



#include "../menu_abstract.h"

#include "KraftKontrol/utils/topic_subscribers.h"



/**
 * This menu shows on the display a list of other menus that can be entered
 */
class Menu_List: public Menu_Abstract {
private:

    //List of menus to show
    List<Menu_Abstract*> menuList_;

    //Control for menu
    Simple_Subscriber<eMenuControlInput_t> menuControlSubscriber_;

    //Which menu in list the cursor points to.
    uint32_t menuCursor_ = 0;

    MenuControl_Abstract& menuControl_;


public:

    /**
     * @param menuControl Which device to use as control for menu.
     */
    Menu_List(const char* menuListName, MenuControl_Abstract& menuControl);

    /**
     * Adds menu to list that will be shown for selection.
     * Adding a list twice will not work. Only one will be shown.
     * Lists will be shown in order they are given.
     * @param menu Menu to add to list.
     */
    void addMenuToList(Menu_Abstract& menu);

    /**
     * Removes given menu from list.
     * @param menu Menu to be removed from list.
     */
    void removeMenuFromList(Menu_Abstract& menu);


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


    void resetMenu() override;


};



#endif