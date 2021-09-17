#ifndef GUI_H
#define GUI_H



#include "KraftKontrol/modules/hid_modules/display_modules/display_abstract.h"

#include "KraftKontrol/utils/list.h"
#include "KraftKontrol/utils/Simple-Schedule/task_autorun_class.h"

#include "menu_abstract.h"



class Gui: public Task_Abstract {
private:

    //Save the path through menus. When entering a new menu, the menu gets added to list and last element is displayed. When exiting, the menu will be removed from list.
    List<Menu_Abstract*> menuPath_;

    //Reference to display used by gui.
    Display_Abstract& display_;

    //Pointer to main menu/ hame page
    Menu_Abstract* mainMenu_;


public:

    Gui(Display_Abstract& display, Menu_Abstract& mainMenu);

    /**
     * initialises display and main menu.
     */
    void init() override;

    /**
     * Gives display and menu time to update.
     */
    void thread() override;

    /**
     * Returns to main menu for user.
     */
    void returnToMainMenu();


};



#endif