#pragma once
#include <ESP32Encoder.h>
#include <Arduino.h>
#include <cstdint>

class EngineDriver {
    private:
    
    // ********** PWM **********
    static constexpr uint16_t freq = 1000; 
    static constexpr uint8_t resolution = 8;
    
    uint8_t pin_front_engine;
    uint8_t pin_back_engine;
    uint8_t pwm_pin;
    uint8_t pwm_channel;

    ESP32Encoder encoder;
    bool encoderSetUp = false;
    
    public:
    EngineDriver() = default;

    bool setUpEngine(uint8_t front_pin, uint8_t back_pin,uint8_t pwm_pin,uint8_t pwm_channel);

    bool setupEncoder(int aPin, int bPin);
    void turnOn(int16_t vel);
    void turnOff();

    bool isEncoderSetUp();

    ~EngineDriver();
};
