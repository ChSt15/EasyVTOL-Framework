#include "st7735_driver.h"



void ST7735Driver::drawString(String string, int16_t x, int16_t y) {

    if (x < 0) x = display_.getCursorX();
    if (y < 0) y = display_.getCursorY() + display_.fontHeight() + 1;

    if (x > display_.width()) x = 0;
    if (y > 160 - display_.fontHeight()) y = 0;

    display_.setCursor(x, y);
    display_.drawString(string, x, y);
    //display_.print(string);

}


void ST7735Driver::clearDisplay() {

    display_.fillScreen(ST7735_BLACK);

}


void ST7735Driver::drawBitmap(uint8_t* greyscaleBitmap, int16_t x, int16_t y, int16_t width, int16_t height) {

    display_.drawBitmap(x, y, greyscaleBitmap, width, height, ST7735_WHITE);

}


void ST7735Driver::thread() {

    if (block_) return;

    loopCounter_++;


    if (moduleStatus_ == eModuleStatus_t::eModuleStatus_Running) {

        //_getData();

    } else if (moduleStatus_ == eModuleStatus_t::eModuleStatus_NotStarted || moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) {
        
        init();

    } else if (false/*moduleStatus_ == eModuleStatus_t::MODULE_CALIBRATING*/) {

        //################## Following is Temporary #################
        

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running; 

    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;
        block_ = true;
        loopRate_ = 0;

    }



    if (_rateCalcInterval.isTimeToRun()) {
        loopRate_ = loopCounter_;
        updateRate_ = updateCounter_;
        updateCounter_ = 0;
        loopCounter_ = 0;
    }

}



void ST7735Driver::init() {

    if (true) {
        
        pinMode(backlightPin_, OUTPUT);
        digitalWrite(backlightPin_, HIGH);

        display_.init(INITR_BLACKTAB);
        display_.setRotation(3);

        lastMeasurement_ = micros();

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running;

        Serial.println("Display Start Success.");

    } else {
        moduleStatus_ = eModuleStatus_t::eModuleStatus_RestartAttempt; 
        Serial.println("Display Start Fail.");
    }

    startAttempts_++;

    if (startAttempts_ >= 5 && moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;

}
