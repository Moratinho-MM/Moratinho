#include "BluetoothHandler.h"

// Static member definitions for ServerState
std::atomic<bool> BluetoothHandler::ServerState::deviceConnected{false};
BLEServer* BluetoothHandler::ServerState::pServer = nullptr;
BLECharacteristic* BluetoothHandler::ServerState::pCharacteristic = nullptr;
bool BluetoothHandler::ServerState::getConnectionState() { return deviceConnected; }


void BluetoothHandler::ServerState::setConnectionState(bool state) { deviceConnected = state; }
void BluetoothHandler::ServerState::setServer(BLEServer *server) { pServer = server; }
void BluetoothHandler::ServerState::setCharacteristic(BLECharacteristic* characteristic){ pCharacteristic = characteristic; }
BLEServer *BluetoothHandler::ServerState::getServer() { return pServer; }

BLECharacteristic *BluetoothHandler::ServerState::getBluetoothCharacteristics() 
{
  return pCharacteristic;
}

void BluetoothHandler::MyServerCallbacks::onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param)
{
  pServer->updatePeerMTU(param->connect.conn_id, 517);
  ServerState::setConnectionState(true);
};

void BluetoothHandler::MyServerCallbacks::onDisconnect(BLEServer *pServer)
{
  ServerState::setConnectionState(false);
  if (pServer) {
    pServer->startAdvertising();
  }
}

void BluetoothHandler::MyCallbacks::onWrite(BLECharacteristic *pCharacteristic) 
{
  std::string rxValue = pCharacteristic->getValue();
  if (rxValue.length() > 0) {
    // Apenas imprime o que recebeu. O envio agora é feito no loop().

    // Se receber o comando de parada, pode executar alguma ação no hardware
    if (rxValue == "halt") 
    {
      // Will be implemented later
    }
  }
}

BLEServer* BluetoothHandler::setUpBluetoothServer()
{
    if (ServerState::getServer() != nullptr)
      return ServerState::getServer();

    BLEServer* pServer = BLEDevice::createServer();

    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    BLECharacteristic* pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_NOTIFY
                    );

    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setCallbacks(new MyCallbacks());
    
    ServerState::setCharacteristic(pCharacteristic);
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();
 
    return pServer;
}

void BluetoothHandler::notifyClient(const String& msg)
{
  if (ServerState::getBluetoothCharacteristics() == nullptr)
    return;
  
  ServerState::getBluetoothCharacteristics()->setValue(msg.c_str());
  ServerState::getBluetoothCharacteristics()->notify();
}

String BluetoothHandler::formatTxMsg(const char* msg, MessageTypes type)
{
    char buffer[256];

    std::snprintf(
        buffer,
        sizeof(buffer),
        "{\"payload\":{\"data_id\":%d,\"message\":\"%s\"}}",
        type, msg
    );
    return String(buffer);
}

String BluetoothHandler::formatSensorDataMsg(float distanceData[3], float yaw)
{
  char buffer[256]; 

  std::snprintf(
    buffer, 
    sizeof(buffer), 
    "{\"payload\":{\"data_id\":%d,\"f_distance\": %f,\"r_distance\": %f,\"l_distance\": %f,\"yaw\": %f}}",
    TELEMETRY_DATA,
    distanceData[0], distanceData[1], distanceData[2],
    yaw
  );

  return String(buffer);
}

bool BluetoothHandler::timeOut()
{
    static unsigned long lastNotifyTime = 0;
    if (millis() - lastNotifyTime >= BLUETOOTH_NOTIFY_TIMER)
    {
        lastNotifyTime = millis();
        return true;
    }
    return false;
}