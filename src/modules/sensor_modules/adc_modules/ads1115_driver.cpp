#include "KraftKontrol/modules/sensor_modules/adc_modules/ads1115_driver.h"



ADS1115Driver::ADS1115Driver(TwoWire& i2cBus, uint32_t rate) : Task_Abstract("ADS1115 Driver", rate*4, eTaskPriority_t::eTaskPriority_VeryHigh), i2cBus_(i2cBus), adc_(0x48) {
    
}


ADCChannel& ADS1115Driver::operator[](uint32_t channel) {
    return adcChannels_[constrain(channel, 0, 4)];
}


void ADS1115Driver::_getData() {

    if (adc_.isBusy()) return;

    //adc_.readADC(currentPin_);

    DataTimestamped<float> adcValue(adc_.toVoltage(adc_.getValue()), NOW());
    adcChannels_[currentPin_].setValue(adcValue);

    currentPin_++;
    if (currentPin_ >= 4) currentPin_ = 0;

    adc_.requestADC(currentPin_);

}


void ADS1115Driver::thread() {


    if (moduleStatus_ == eModuleStatus_t::eModuleStatus_Running) {

        _getData();

    } else if (moduleStatus_ == eModuleStatus_t::eModuleStatus_NotStarted || moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) {
        
        init();

    } else if (false/*moduleStatus_ == eModuleStatus_t::MODULE_CALIBRATING*/) {

        //################## Following is Temporary #################
        

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running; 

    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;
        stopTaskThreading();

    }


}



void ADS1115Driver::init() {

    int startCode = adc_.begin();

    if (startCode > 0) {

        adc_.setMode(1);
        adc_.setGain(1);
        adc_.setDataRate(7);
        adc_.requestADC(0);

        lastMeasurement_ = micros();

        moduleStatus_ = eModuleStatus_t::eModuleStatus_Running;

        Serial.println("ADC Start Success.");

        i2cBus_.setClock(1000000);

    } else {
        moduleStatus_ = eModuleStatus_t::eModuleStatus_RestartAttempt; 
        Serial.println("ADC Start Fail. Code: " + String(startCode));
    }

    startAttempts_++;

    if (startAttempts_ >= 5 && moduleStatus_ == eModuleStatus_t::eModuleStatus_RestartAttempt) moduleStatus_ = eModuleStatus_t::eModuleStatus_Failure;

}
