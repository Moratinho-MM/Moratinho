#pragma once

class ESP32Encoder {
public:
    ESP32Encoder() = default;
    void attachHalfQuad(int pinA, int pinB) {}
    void clearCount() {}
    long getCount() const { return 0; }
};
