#ifndef HBRIDGE_ABSTRACT
#define HBRIDGE_ABSTRACT



class HBridge_Abstract {
private:


public:

    /**
     * Sets the power of the output. Negative values will reverse the output.
     * @param power Output power in from -1 to 1. 
     */
    virtual void setOutput(float power) = 0;

    /**
     * @returns current output power from -1 to 1.
     */
    virtual float getOutput() = 0;


};

#endif