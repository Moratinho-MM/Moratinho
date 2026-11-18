#include "EngineDriver.h"

bool EngineDriver::setUpEngine(uint8_t front_pin, uint8_t back_pin,uint8_t pwm_pin,uint8_t pwm_channel)
{
    pin_front_engine = front_pin;
    pin_back_engine = back_pin;
    this->pwm_pin = pwm_pin;
    this->pwm_channel= pwm_channel;

    pinMode(pin_front_engine, OUTPUT);
    pinMode(pin_back_engine, OUTPUT);

    //** configuracao_PWM ** 
    if (!ledcSetup(pwm_channel, freq, resolution))
        return false;
    
    ledcAttachPin(pwm_pin, pwm_channel);
    
    return true;
}

bool EngineDriver::setupEncoder(int aPin, int bPin)
{
    encoder.attachHalfQuad(
        aPin,
        bPin
    );
    encoder.pauseCount();
    encoder.clearCount();
    encoder.resumeCount();
    encoderSetUp = true;
    return true;
}
void EngineDriver::turnOn(int16_t vel)
{
    
    if (vel > 0){
        digitalWrite(pin_front_engine,1);
        digitalWrite(pin_back_engine,0);   
    }
    else{
        vel=vel*(-1);
        digitalWrite(pin_front_engine,0);
        digitalWrite(pin_back_engine,1);
    }
    ledcWrite(pwm_channel,vel); 
}
void EngineDriver::turnOff()
{
    ledcWrite(pwm_channel,0);
    digitalWrite(pin_front_engine,0);
    digitalWrite(pin_back_engine,0);   
}

void EngineDriver::brake()
{
    digitalWrite(pin_front_engine, 1);
    digitalWrite(pin_back_engine, 1);
    ledcWrite(pwm_channel, 255);
}

bool EngineDriver::isEncoderSetUp()
{
    return encoderSetUp;
}

EngineDriver::~EngineDriver() 
{
    encoder.detach();
    turnOff();
    ledcDetachPin(pwm_pin);
}

int EngineDriver::getEncoderMeasure()
{
    return encoder.getCount();
}

void EngineDriver::resetEncoderMeasure()
{
    encoder.clearCount();
}
