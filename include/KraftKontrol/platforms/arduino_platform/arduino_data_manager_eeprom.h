#ifndef DATA_MANAGER_ARDUINO_EEPROM_H
#define DATA_MANAGER_ARDUINO_EEPROM_H



#include "../../modules/data_manager_modules/data_manager_nonvolatile.h"

#include "EEPROM.h"


/**
 * This class inherets from DataManager_Abstract and can be used as such. @see DataManager_Abstract. 
 * This class is used to save and load data from an Arduino boards internal EEPROM data. 
 */
class DataManager_InternalEEPROM final: public DataManager_NonVolatile {
private:

    ///Pointer to data begin. Must be allocated on construction.
    uint8_t* dataPointer_ = nullptr;
    ///Size of the allocated data.
    uint32_t dataSize_ = 0;


public:

    /**
     * @param dataSize Number of bytes the storage should be.
     */
    DataManager_InternalEEPROM();

    ~DataManager_InternalEEPROM();

public:

    /**
     * Reads data from data storage.
     * @param index First byte to start reading from.
     * @param data Pointer to data where to write data to.
     * @param numBytes Number of bytes to read.
     * @returns true if successfull.
     */
    bool readData(uint32_t index, uint8_t* data, uint32_t numBytes = 1) const override;

    /**
     * Writes data to data storage.
     * @param index First byte to start writting data to.
     * @param data Pointer to data that should be written to data storage.
     * @param numBytes Number of bytes to write.
     * @returns true if successfull.
     */
    bool writeData(uint32_t index, const uint8_t* data, uint32_t numBytes = 1) override;

    /**
     * @returns end index of memory.
     */
    uint32_t getEndIndex() const override;


public:

    /**
     * Copies EEPROM data into RAM. Loading data from EEPROM take a long time.
     * @returns true if successfull.
     */
    bool loadData() override;

    /**
     * Places RAM data into EEPROM.
     * This will only update changes bytes in EEPROM to increase lifetime. It also takes a while to save everything.
     * Warning: calling this function write to EEPROM which decreases lifetime. Only call this if you actually need to save the memory.
     * @returns true if successfull.
     */
    bool saveData() override;
    
};



#endif