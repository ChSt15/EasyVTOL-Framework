#include "KraftKontrol/gui/menu_abstract.h"



Menu_Abstract::Menu_Abstract(const char* menuName) {
    
    setMenuName(menuName);

}


void Menu_Abstract::setMenuName(const char* menuName) {
    strncpy(menuName_, menuName, MENU_MAX_NAME_SIZE);
}


const char* Menu_Abstract::getMenuName() {return menuName_;}


