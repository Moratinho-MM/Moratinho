#include "BluetoothHandler.h"
#include "../MovController/MovController.h"
#include "../VL530X_Goytacaz/VL530X_Goytacaz.h"
#include <nlohmann/json.hpp>

// Static member definitions for ServerState
std::atomic<bool> BluetoothHandler::ServerState::deviceConnected{false};
BLEServer* BluetoothHandler::ServerState::pServer = nullptr;
BLECharacteristic* BluetoothHandler::ServerState::pCharacteristic = nullptr;

static MovController* s_pController = nullptr;
static IRSensors* s_pIrSensors = nullptr;

static std::atomic<BluetoothHandler::PendingCommand> s_pendingCmd{BluetoothHandler::PendingCommand::NONE};
static std::atomic<float> s_pendingParam{0.0f};
static std::atomic<int> s_pendingCount{1};
static std::atomic<bool> s_indeterminateMoving{false};
static std::atomic<bool> s_startRequested{false};
bool BluetoothHandler::ServerState::getConnectionState() { return deviceConnected; }

void BluetoothHandler::ServerState::setConnectionState(bool state) { deviceConnected = state; }
void BluetoothHandler::ServerState::setServer(BLEServer *server) { pServer = server; }
void BluetoothHandler::ServerState::setCharacteristic(BLECharacteristic* characteristic){ pCharacteristic = characteristic; }
BLEServer *BluetoothHandler::ServerState::getServer() { return pServer; }

static bool recivedResponse = false;
static bool lastResponse = false;

bool BluetoothHandler::getLastResponse()
{
    recivedResponse = false;
    return lastResponse;
}
bool BluetoothHandler::hasRecivedRespone()
{
    return recivedResponse;
}


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

void BluetoothHandler::init(MovController* controller, IRSensors* irSensors)
{
    s_pController = controller;
    s_pIrSensors = irSensors;
}

bool BluetoothHandler::isIndeterminateMoving()
{
    return s_indeterminateMoving.load();
}

bool BluetoothHandler::checkAndClearStartRequest()
{
    bool expected = true;
    return s_startRequested.compare_exchange_strong(expected, false);
}

void BluetoothHandler::notifyCurrentParams()
{
    if (!s_pController) return;

    int16_t offsetL = 0, offsetF = 0, offsetR = 0;
    if (s_pIrSensors) {
        offsetL = s_pIrSensors->getLeftOffsetMm();
        offsetF = s_pIrSensors->getFrontOffsetMm();
        offsetR = s_pIrSensors->getRightOffsetMm();
    }

    char buffer[256];
    std::snprintf(
        buffer,
        sizeof(buffer),
        "kp:%.3f, ki:%.3f, kd:%.3f,wall_kp:%.3f,bias:%d,turn_bias:%.1f,offset_l:%d,offset_f:%d,offset_r:%d,angle_offset:%.1f,motor_d:%d,motor_e:%d",
        s_pController->getKp(),
        s_pController->getKi(),
        s_pController->getKd(),
        s_pController->getWallKp(),
        s_pController->getBias(),
        s_pController->getTurnBias(),
        offsetL, offsetF, offsetR,
        s_pController->getAngleOffset(),
        s_pController->getMotorDOffset(),
        s_pController->getMotorEOffset()
    );
    notifyClient(formatTxMsg(buffer, MSG));
}

void BluetoothHandler::handleIncomingJson(const std::string& rxValue)
{
    // Raw emergency halt shortcuts
    if (rxValue == "halt" || rxValue == "stop") {
        s_indeterminateMoving.store(false);
        s_pendingCmd.store(PendingCommand::STOP);
        if (s_pController) {
            s_pController->brake();
            s_pController->stop();
        }
        notifyClient(formatTxMsg("mouse halted!",WARNING));
        return;
    }

    nlohmann::json j;
    try {
        j = nlohmann::json::parse(rxValue);
    } catch (const std::exception& e) {
        notifyClient(formatTxMsg("json_parser_error", WARNING));
        return;
    }

    // 1. Parameter Tuning Updates
    if (s_pController) {
        bool paramsUpdated = false;
        float kp = s_pController->getKp();
        float ki = s_pController->getKi();
        float kd = s_pController->getKd();
        float wallKp = s_pController->getWallKp();
        int bias = s_pController->getBias();
        float turnBias = s_pController->getTurnBias();
        float angleOffset = s_pController->getAngleOffset();
        int motorD = s_pController->getMotorDOffset();
        int motorE = s_pController->getMotorEOffset();

        int16_t offsetL = 0, offsetF = 0, offsetR = 0;
        if (s_pIrSensors) {
            offsetL = s_pIrSensors->getLeftOffsetMm();
            offsetF = s_pIrSensors->getFrontOffsetMm();
            offsetR = s_pIrSensors->getRightOffsetMm();
        }

        if (j.contains("kp")) { kp = j["kp"].get<float>(); paramsUpdated = true; }
        if (j.contains("ki")) { ki = j["ki"].get<float>(); paramsUpdated = true; }
        if (j.contains("kd")) { kd = j["kd"].get<float>(); paramsUpdated = true; }
        if (j.contains("wall_kp")) { wallKp = j["wall_kp"].get<float>(); paramsUpdated = true; }
        if (j.contains("kp_wall")) { wallKp = j["kp_wall"].get<float>(); paramsUpdated = true; }
        if (j.contains("bias")) { bias = j["bias"].get<int>(); paramsUpdated = true; }
        if (j.contains("linear_bias")) { bias = j["linear_bias"].get<int>(); paramsUpdated = true; }
        if (j.contains("turn_bias")) { turnBias = j["turn_bias"].get<float>(); paramsUpdated = true; }
        if (j.contains("angle_offset")) { angleOffset = j["angle_offset"].get<float>(); paramsUpdated = true; }
        if (j.contains("motor_d")) { motorD = j["motor_d"].get<int>(); paramsUpdated = true; }
        if (j.contains("motor_e")) { motorE = j["motor_e"].get<int>(); paramsUpdated = true; }

        if (j.contains("offset_l")) { offsetL = j["offset_l"].get<int16_t>(); paramsUpdated = true; }
        if (j.contains("left_offset")) { offsetL = j["left_offset"].get<int16_t>(); paramsUpdated = true; }
        if (j.contains("offset_f")) { offsetF = j["offset_f"].get<int16_t>(); paramsUpdated = true; }
        if (j.contains("front_offset")) { offsetF = j["front_offset"].get<int16_t>(); paramsUpdated = true; }
        if (j.contains("offset_r")) { offsetR = j["offset_r"].get<int16_t>(); paramsUpdated = true; }
        if (j.contains("right_offset")) { offsetR = j["right_offset"].get<int16_t>(); paramsUpdated = true; }

        if (paramsUpdated) {
            s_pController->setPid(kp, ki, kd);
            s_pController->setWallKp(wallKp);
            s_pController->setBias(bias);
            s_pController->setTurnBias(turnBias);
            s_pController->setAngleOffset(angleOffset);
            s_pController->setMotorOffsets(motorD, motorE);
            if (s_pIrSensors) {
                s_pIrSensors->setOffsetsMm(offsetL, offsetF, offsetR);
            }
            notifyCurrentParams();
        }
    }

    // 2. Command Processing
    if (j.contains("cmd")) {
        std::string cmd = j["cmd"].get<std::string>();

        if (cmd == "get_params" || cmd == "params") {
            notifyCurrentParams();
        }
        else if (cmd == "turn" || cmd == "turn_deg" || cmd == "turn_degrees") {
            float angle = 0.0f;
            if (j.contains("angle")) angle = j["angle"].get<float>();
            else if (j.contains("deg")) angle = j["deg"].get<float>();
            else if (j.contains("degrees")) angle = j["degrees"].get<float>();

            s_pendingParam.store(angle);
            s_pendingCmd.store(PendingCommand::TURN_DEG);
            
            char buffer[50];
            std::snprintf(
                buffer,
            sizeof(buffer),
                "status : queued,cmd : turn_deg,angle : %.1f",
                angle
            );
            notifyClient(formatTxMsg(buffer, MSG));
        }
        else if (cmd == "turn_right") {
            s_pendingCmd.store(PendingCommand::TURN_RIGHT);
            notifyClient(formatTxMsg("status : queued,cmd : turn_right", MSG));
        }
        else if (cmd == "turn_left") {
            s_pendingCmd.store(PendingCommand::TURN_LEFT);
            notifyClient(formatTxMsg("status : queued,cmd : turn_left", MSG));
        }
        else if (cmd == "turn_180" || cmd == "turn_180degrees") {
            s_pendingCmd.store(PendingCommand::TURN_180);
            notifyClient(formatTxMsg("status : queued,cmd : turn_180", MSG));
        }
        else if (cmd == "move_cell") {
            int count = 1;
            if (j.contains("count")) count = j["count"].get<int>();
            else if (j.contains("cells")) count = j["cells"].get<int>();

            if (count > 1) {
                s_pendingCount.store(count);
                s_pendingCmd.store(PendingCommand::MOVE_CELLS);
                
                char buffer[50];
                std::snprintf(buffer, sizeof(buffer),
                    "status : queued, cmd : move_cells, count : %d",
                    count
                );
                notifyClient(formatTxMsg(buffer, MSG));
            } else {
                s_pendingCmd.store(PendingCommand::MOVE_CELL);
                notifyClient(formatTxMsg("status : queued, cmd : move_cell", MSG));
            }
        }
        else if (cmd == "move_cells") {
            int count = 1;
            if (j.contains("count")) count = j["count"].get<int>();
            else if (j.contains("cells")) count = j["cells"].get<int>();

            s_pendingCount.store(count);
            s_pendingCmd.store(PendingCommand::MOVE_CELLS);
            
            char buffer[50];
            std::snprintf(buffer, sizeof(buffer),
                "status : queued,cmd : move_cells,count : %d",
                count
            );
            notifyClient(formatTxMsg(buffer, MSG));
        }
        else if (cmd == "move_forward" || cmd == "move_front") {
            s_indeterminateMoving.store(true);
            s_pendingCmd.store(PendingCommand::MOVE_FORWARD_INDETERMINATE);
            notifyClient(formatTxMsg("status : running,cmd : move_forward", MSG));
        }
        else if (cmd == "stop" || cmd == "halt") {
            s_indeterminateMoving.store(false);
            s_pendingCmd.store(PendingCommand::STOP);
            if (s_pController) {
                s_pController->brake();
                s_pController->stop();
            }
            notifyClient(formatTxMsg("status : ok,action : stopped", MSG));
        }
        else if (cmd == "start_explore" || cmd == "start") {
            s_startRequested.store(true);
            notifyClient(formatTxMsg("status : ok,action : start_explore_triggered", MSG));
        }
        else {
            notifyClient(formatTxMsg("status : error,msg : unknown_command", WARNING));
        }
    }

    if (j.contains("res")) {
        bool res = j["res"].get<bool>();

        lastResponse = res;
        recivedResponse = true;
    }
}

void BluetoothHandler::processPendingCommands()
{
    if (!s_pController) return;

    // Handle continuous indeterminate forward motion
    if (s_indeterminateMoving.load()) {
        s_pController->moveFront();
        if (s_pController->isFrontEmergencyStop()) {
            s_pController->brake();
            s_pController->stop();
            s_indeterminateMoving.store(false);
            notifyClient(formatTxMsg("status : stopped,reason : front_obstacle", WARNING));
        }
        return;
    }

    PendingCommand cmd = s_pendingCmd.exchange(PendingCommand::NONE);
    if (cmd == PendingCommand::NONE) {
        return;
    }

    float angle = s_pendingParam.load();
    int count = s_pendingCount.load();

    switch (cmd) {
        case PendingCommand::TURN_DEG: {
            s_pController->turnDegrees(angle);
            char buffer[50];
            std::snprintf(buffer, sizeof(buffer), "status : done,cmd : turn_deg,angle : %.1f", angle);
            notifyClient(formatTxMsg(buffer, MSG));
            break;
        }
        case PendingCommand::TURN_RIGHT:
            s_pController->turnRight();
            notifyClient(formatTxMsg("status : done, cmd : turn_right", MSG));
            break; 
        case PendingCommand::TURN_LEFT:
            s_pController->turnLeft();
            notifyClient(formatTxMsg("status : done, cmd : turn_left", MSG));
            break ;
        case PendingCommand::TURN_180:
            s_pController->turn180degrees();
            notifyClient(formatTxMsg("status : done, cmd : turn_180", MSG));
            break;
        case PendingCommand::MOVE_CELL:
            s_pController->moveCell();
            notifyClient(formatTxMsg("status : done,cmd : move_cell", MSG));
            break;
        case PendingCommand::MOVE_CELLS: {
            s_pController->moveCells(count);
            char buffer[50];
            std::snprintf(buffer, sizeof(buffer), "status : done,cmd : move_cells,count : %d", count);
            notifyClient(formatTxMsg(buffer, MSG));
            break;
        }
        case PendingCommand::STOP:
            s_pController->brake();
            s_pController->stop();
            s_indeterminateMoving.store(false);
            notifyClient(formatTxMsg("status : done,cmd : stop", MSG));
            break;
        default:
            break;
    }
}

void BluetoothHandler::MyCallbacks::onWrite(BLECharacteristic *pCharacteristic)
{
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
        handleIncomingJson(rxValue);
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