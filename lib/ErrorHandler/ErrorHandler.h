#pragma once

#include "BluetoothHandler.h"

class ErrorHandler {
    public:

    static void throwError(const char* msg, const char* emitter);
    static void throwWarning(const char* msg, const char* emitter);
};