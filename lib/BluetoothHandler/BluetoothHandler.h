#pragma once

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino.h>
#include <atomic>
#include "../ConstantConfigs.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


namespace BluetoothHandler 
{
    typedef enum {
        MSG,
        FATAL_ERROR,
        WARNING,
        IR_DATA,
        MPU_DATA,
        TELEMETRY_DATA,
        F_WALL_REQ,
        L_WALL_REQ,
        R_WALL_REQ,
        MOV_F_REQ,
        TURN_R_REQ,
        TURN_L_REQ,
        SET_CELL_D_REQ,
        CLEAR_CELL_D_REQ,
    } MessageTypes;


    class ServerState
    {
        private:

        static std::atomic<bool> deviceConnected;
        static BLEServer* pServer;
        static BLECharacteristic* pCharacteristic;

        public:

        static bool getConnectionState();
        static bool getOldDeviceState();
        static void setConnectionState(bool state);
        static void updateOldDeviceState(bool state);
        
        static void setServer(BLEServer* server);
        static void setCharacteristic(BLECharacteristic* pCharacteristic);
        static BLEServer* getServer();
        static BLECharacteristic* getBluetoothCharacteristics();
        
    };

    class MyServerCallbacks: public BLEServerCallbacks
    {   
        public:
        void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param);

        void onDisconnect(BLEServer* pServer);
    };

    class MyCallbacks: public BLECharacteristicCallbacks 
    {
        void onWrite(BLECharacteristic *pCharacteristic);
    };

    BLEServer* setUpBluetoothServer();

    void notifyClient(const String& msg);
    String formatTxMsg(const char* msg, MessageTypes type);
    String formatSensorDataMsg(float distanceData[3], float yaw);

    bool timeOut();
}