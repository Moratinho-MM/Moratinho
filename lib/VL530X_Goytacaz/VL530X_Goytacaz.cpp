#include "VL530X_Goytacaz.h"

// Sensible bounds for a micromouse cell (180 mm outer / ~168 mm inner wall-to-wall)
static constexpr uint16_t MIN_VALID_MM = 15;     // Closer than 1.5 cm is unphysical / noise
static constexpr uint16_t MAX_VALID_MM = 1200;   // VL53L0X max reliable range in cell
static constexpr float OUT_OF_RANGE_CM = 100.0f; // Safe value indicating no wall

void IRSensors::begin(int xshut_f, int xshut_r, int xshut_l)
{
    const char* emitter = "VL530X_Goytacaz.h";

    Wire.begin();
    Wire.setClock(400000); // 400kHz fast I2C

    pinMode(xshut_r, OUTPUT);
    pinMode(xshut_f, OUTPUT);
    pinMode(xshut_l, OUTPUT);

    // Shutdown all sensors so all start off the default 0x29 address
    digitalWrite(xshut_r, LOW);
    digitalWrite(xshut_f, LOW);
    digitalWrite(xshut_l, LOW);
    delay(10);

    // Bring up sensors one by one and assign distinct I2C addresses
    digitalWrite(xshut_r, HIGH);
    delay(10);
    if (!rightIfs.init()) {
        ErrorHandler::throwError("Failed to initialize right side irSensor", emitter);
    }
    rightIfs.setAddress(0x30);

    digitalWrite(xshut_f, HIGH);
    delay(10);
    if (!frontIfs.init()) {
        ErrorHandler::throwError("Failed to initialize frontal irSensor", emitter);
    }
    frontIfs.setAddress(0x31);

    digitalWrite(xshut_l, HIGH);
    delay(10);
    if (!leftIfs.init()) {
        ErrorHandler::throwError("Failed to initialize left side irSensor", emitter);
    }
    leftIfs.setAddress(0x32);

    rightIfs.setTimeout(35);
    frontIfs.setTimeout(35);
    leftIfs.setTimeout(35);

    // 20 ms timing budget in continuous back-to-back ranging mode
    rightIfs.setMeasurementTimingBudget(20000);
    frontIfs.setMeasurementTimingBudget(20000);
    leftIfs.setMeasurementTimingBudget(20000);

    rightIfs.startContinuous();
    frontIfs.startContinuous();
    leftIfs.startContinuous();
}

void IRSensors::setOffsetsMm(int16_t leftOffset, int16_t frontOffset, int16_t rightOffset)
{
    leftOffsetMm  = leftOffset;
    frontOffsetMm = frontOffset;
    rightOffsetMm = rightOffset;
}

float IRSensors::readFilteredDistance(VL53L0X& sensor, float& lastValid, int16_t offsetMm, bool& didUpdate)
{
    didUpdate = false;

    // Non-blocking data-ready poll on RESULT_INTERRUPT_STATUS (bits [2:0] != 0 when ready)
    uint8_t intStatus = sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
    if (sensor.last_status != 0 || (intStatus & 0x07) == 0) {
        // No new sample ready yet; reuse cached distance without stalling the PID loop
        return lastValid;
    }

    // Read range register directly and clear interrupt flag (non-blocking equivalent of readRangeContinuousMillimeters)
    uint16_t raw_mm = sensor.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    sensor.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);

    if (sensor.last_status != 0) {
        return lastValid;
    }

    didUpdate = true;

    // Check if reading is out-of-bounds / open corridor
    if (raw_mm == 65535 || raw_mm > MAX_VALID_MM) {
        lastValid = OUT_OF_RANGE_CM;
        return lastValid;
    }

    // Apply static offset calibration
    int32_t calibrated_mm = static_cast<int32_t>(raw_mm) - static_cast<int32_t>(offsetMm);
    if (calibrated_mm < 0) {
        calibrated_mm = 0;
    }

    if (calibrated_mm < MIN_VALID_MM) {
        // Discard 0mm glitch by retaining previous valid reading
        return lastValid;
    }

    lastValid = calibrated_mm / 10.0f; // Convert mm to cm
    return lastValid;
}

float IRSensors::getRightDistance() 
{
    return readFilteredDistance(rightIfs, lastRightDist, rightOffsetMm, rightUpdated);
}

float IRSensors::getFrontDistance() 
{
    return readFilteredDistance(frontIfs, lastFrontDist, frontOffsetMm, frontUpdated);
}

float IRSensors::getLeftDistance() 
{
    return readFilteredDistance(leftIfs, lastLeftDist, leftOffsetMm, leftUpdated);
}

float IRSensors::getRightDistanceMm()
{
    return getRightDistance() * 10.0f;
}

float IRSensors::getFrontDistanceMm()
{
    return getFrontDistance() * 10.0f;
}

float IRSensors::getLeftDistanceMm()
{
    return getLeftDistance() * 10.0f;
}

void IRSensors::showDistances() 
{
    // Cached non-blocking values in cm
    (void)getRightDistance();
    (void)getFrontDistance();
    (void)getLeftDistance();
}