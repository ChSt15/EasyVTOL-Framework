#include "KraftKontrol/gui/menus/menu_test.h"




Menu_Test::Menu_Test(const char* menuListName, MenuControl_Abstract& menuControl): Menu_Abstract(menuListName) {

    menuControlSubscriber_.subscribe(menuControl.getMenuInputTopic());

}


Menu_Abstract* Menu_Test::menuDisplayUpdate(Display_Abstract& display, Task_Abstract& gui) {

    Menu_Abstract* returnValue = this;

    //Handle inputs first
    while(menuControlSubscriber_.available()) {

        eMenuControlInput_t controlInput;
        menuControlSubscriber_.takeBack(controlInput);

        switch (controlInput) {
        case eMenuControlInput_t::eMenuControlInput_LeftLong: //Leave menu.
            returnValue = nullptr;
            break;
        
        default:
            break;
        }

    }


    //Draw menu status

    Color color(255);

    display.clear();

    display.drawText(20, 20, color, 1, getMenuName());

    display.drawText(display.getDisplayWidth()/2, display.getDisplayHeight()/2, color, 2, "TEST DISPLAY!");

    display.update();

    return returnValue;

}
