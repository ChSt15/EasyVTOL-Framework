#ifndef DATA_MANAGER_NONVOLATILE_H
#define DATA_MANAGER_NONVOLATILE_H



#include "stdint.h"

#include "data_manager_abstract.h"



/**
 * This class inherets from DataManager_Abstract and can be used as such. @see DataManager_Abstract. 
 * This class is used to save and load data from non volatile memory like EEPROM, sdcards etc.
 */
class DataManager_NonVolatile: public DataManager_Abstract {
public:


    /**
     * Copies non volatile memory data into RAM. Loading data can take a long time.
     * @returns true if successfull.
     */
    virtual bool loadData() = 0;

    /**
     * Places RAM data into non volatile memory.
     * @returns true if successfull.
     */
    virtual bool saveData() = 0;
    
};



#endif