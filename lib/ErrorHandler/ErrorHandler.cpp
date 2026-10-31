#include "ErrorHandler.h"
#include "../ConstantConfigs.h"
#include "../EngineDriver/EngineDriver.h"
#include <Arduino.h>

void ErrorHandler::throwError(const char* msg, const char* emitter)
{
    #ifdef BLUETOOTH_ENABLE
    char fullMsg[128];
    snprintf(fullMsg, sizeof(fullMsg), "Error: %s | Emitter: %s", msg, emitter);
    String formatMsg = BluetoothHandler::formatTxMsg(fullMsg, BluetoothHandler::FATAL_ERROR);
    BluetoothHandler::notifyClient(formatMsg);
    
    delay(500); 
    #else
    Serial.printf("Error: %s | Emitter: %s\n", msg, emitter);
    #endif
    
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Yields to FreeRTOS and prevents Watchdog trigger
    }
}

void ErrorHandler::throwWarning(const char* msg, const char* emitter)
{
    #ifdef BLUETOOTH_ENABLE
    char fullMsg[128];
    snprintf(fullMsg, sizeof(fullMsg), "Warning: %s | Emitter: %s", msg, emitter);
    String formatMsg = BluetoothHandler::formatTxMsg(fullMsg, BluetoothHandler::WARNING);
    BluetoothHandler::notifyClient(formatMsg);
     
    #endif
}