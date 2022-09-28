#include "KraftKontrol/gui/menus/menu_vector.h"


#include "Arduino.h"


Menu_Vector::Menu_Vector(const Topic<VectorOLD<>>& vectorTopic, const char* menuListName, MenuControl_Abstract& menuControl): Menu_Abstract(menuListName) {

    menuControlSubscriber_.subscribe(menuControl.getMenuInputTopic());
    vectorSubr_.subscribe(vectorTopic);

}


Menu_Abstract* Menu_Vector::menuDisplayUpdate(Display_Abstract& display, Task_Threading& gui) {

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

    //display.drawText(10, display.getDisplayHeight()/2, color, 1, (vectorSubr_.getItem().toString()).c_str());

    display.update();

    return returnValue;

}
