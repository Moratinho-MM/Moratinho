#include "VL530X_Goytacaz.h"


void IRSensors::begin(int xshut_f, int xshut_r, int xshut_l)
{
    const char* emitter = "VL530X_Goytacaz.h";

    Wire.begin();

    pinMode(xshut_r, OUTPUT);
    pinMode(xshut_f, OUTPUT);
    pinMode(xshut_l, OUTPUT);

    // Shutdown all sensors
    digitalWrite(xshut_r, LOW);
    digitalWrite(xshut_f, LOW);
    digitalWrite(xshut_l, LOW);
    delay(10);

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

    rightIfs.setTimeout(500);
    frontIfs.setTimeout(500);
    leftIfs.setTimeout(500);

    rightIfs.setMeasurementTimingBudget(20000);
    frontIfs.setMeasurementTimingBudget(20000);
    leftIfs.setMeasurementTimingBudget(20000);

    rightIfs.startContinuous();
    frontIfs.startContinuous();
    leftIfs.startContinuous();
}

float IRSensors::getRightDistance() {
    return rightIfs.readRangeContinuousMillimeters() / 10.0f;
}

float IRSensors::getFrontDistance() {
    return frontIfs.readRangeContinuousMillimeters() / 10.0f;
}

float IRSensors::getLeftDistance() {
    return leftIfs.readRangeContinuousMillimeters() / 10.0f;
}

void IRSensors::showDistances() 
{ // TODO : Mudar a função para usar o sistema de logs ao inves de Serial prints 
    //Serial.print("Sensor 1 (D) (cm): ");
    //Serial.print(rightIfs.readRangeContinuousMillimeters() / 10.0);
    if (rightIfs.timeoutOccurred())
    {
        //Serial.print(" TIMEOUT");
    }

    //Serial.print(" | Sensor 2 (F) (cm): ");
    //Serial.print(frontIfs.readRangeContinuousMillimeters() / 10.0);
    if (frontIfs.timeoutOccurred())
    {
        //Serial.print(" TIMEOUT");
    }

    //Serial.print(" | Sensor 3 (E) (cm): ");
    //Serial.print(leftIfs.readRangeContinuousMillimeters() / 10.0);
    if (leftIfs.timeoutOccurred())
    {
        //Serial.print(" TIMEOUT");
    }

    //Serial.println();
}