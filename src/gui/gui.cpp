#include "KraftKontrol/gui/gui.h"



Gui::Gui(Display_Abstract& display, Menu_Abstract& mainMenu): Task_Abstract("GUI", 30, eTaskPriority_t::eTaskPriority_Middle), display_(display) {
    mainMenu_ = &mainMenu;
}


void Gui::init() {} //Currently nothing to do


void Gui::returnToMainMenu() {
    
    menuPath_.clear();

}


void Gui::thread() {

    Menu_Abstract* menu = menuPath_.getNumItems() > 0 ? menuPath_[menuPath_.getNumItems()-1] : mainMenu_;

    Menu_Abstract* menuReturn = menu->menuDisplayUpdate(display_, *this); //Call display update.

    if (menuReturn == nullptr && menu != mainMenu_) { //Return to last menu. Remove last menu in list and reset gui task settings, like FPS.

        if (menuPath_.getNumItems() > 0 ) menuPath_[menuPath_.getNumItems()-1]->exit();

        menuPath_.removeAtIndex(menuPath_.getNumItems()-1); //Remove last item. Shouldnt be expensive.

        setTaskRate(30);
        setTaskPriority(eTaskPriority_t::eTaskPriority_Middle);

    } else if (menuReturn != menu) { //New menu selected. Add to end of list.

        menuReturn->enter();
        menuPath_.append(menuReturn);

    }

}



