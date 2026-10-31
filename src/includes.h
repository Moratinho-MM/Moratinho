#ifndef INCLUDES_H
#define INCLUDES_H

//********** bibliotecas **********
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Arduino.h>

//********** scripts internos *********
#include "../lib/ConstantConfigs.h"
#include "MPU6050_Goytacaz.h"
#include "VL530X_Goytacaz.h"
#include "../MovController/MovController.h"
#include "../EngineDriver/EngineDriver.h"
#include "Maze.h"
#include "BluetoothHandler.h"
#include "../ErrorHandler/ErrorHandler.h"
#include "../Simulator/Simulator.h"

#endif