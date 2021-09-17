#include "KraftKontrol/gui/menus/menu_list.h"




Menu_List::Menu_List(const char* menuListName, MenuControl_Abstract& menuControl): Menu_Abstract(menuListName), menuControl_(menuControl) {

    menuControlSubscriber_.subscribe(menuControl.getMenuInputTopic());

}


void Menu_List::addMenuToList(Menu_Abstract& menu) {
    menuList_.appendIfNotInList(&menu);
}


void Menu_List::removeMenuFromList(Menu_Abstract& menu) {
    menuList_.removeAllEqual(&menu);
}


void Menu_List::resetMenu() {

    menuControlSubscriber_.subscribe(menuControl_.getMenuInputTopic());

}


Menu_Abstract* Menu_List::menuDisplayUpdate(Display_Abstract& display, Task_Abstract& gui) {

    Menu_Abstract* returnValue = this;

    //Handle inputs first
    if(menuControlSubscriber_.isDataNew()) {

        eMenuControlInput_t controlInput = menuControlSubscriber_.getItem();

        switch (controlInput) {
        case eMenuControlInput_t::eMenuControlInput_UpShort:
            if (menuCursor_ > 0) menuCursor_--;
            break;

        case eMenuControlInput_t::eMenuControlInput_DownShort:
            menuCursor_++;
            break;

        case eMenuControlInput_t::eMenuControlInput_RightShort: //Enter menu.
            returnValue = menuList_[menuCursor_];
            break;

        case eMenuControlInput_t::eMenuControlInput_LeftLong: //Leave menu.
            returnValue = nullptr;
            menuControlSubscriber_.unsubcribe();
            break;
        
        default:
            break;
        }

    }

    menuCursor_ = constrain(menuCursor_, 0.0, menuList_.getNumItems()-1); //Must be done like this in case menus are removed at some point.


    //Draw menu status

    Color color(255);

    display.clear();

    display.drawText(20, 20, color, 1, getMenuName());

    for (uint32_t i = 0; i < menuList_.getNumItems(); i++) {

        display.drawText(10, i*10+30, color, 0, menuList_[i]->getMenuName());

        if (i == menuCursor_) {
            display.drawText(0, i*10+30, color, 0, ">");
        }

    }

    display.update();

    return returnValue;

}
