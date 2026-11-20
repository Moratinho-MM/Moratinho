#include "includes.h"

MovController controller;    
IRSensors irSensors;
IMU giroscope;

EngineDriver leftEngine;
EngineDriver rightEngine;

Maze maze(MAZE_SIZE, MAZE_GOAL_X, MAZE_GOAL_Y);

void setup()
{
    Serial.begin(115200);
    #ifdef BLUETOOTH_ENABLE
    BLEDevice::init(BLUETOOTH_NAME);
    
    BluetoothHandler::ServerState::setServer(
        BluetoothHandler::setUpBluetoothServer()
    );
    #else
    unsigned long timeout_start = millis();
    while (!Serial && (millis() - timeout_start < 1000));
    Serial.println("No bluetooth set : Serial connection has started.");
    #endif
    

    irSensors.begin(
        XSHUT_F,
        XSHUT_R,
        XSHUT_L
    );

    giroscope.begin();
    
    if(!leftEngine.setUpEngine(
        LEFT_ENGINE_FRONT_PIN, 
        LEFT_ENGINE_BACK_PIN,
        LEFT_ENGINE_PWM_PIN, 
        LEFT_ENGINE_PWM_CHANNEL
    ))
    {
        ErrorHandler::throwError("left engine setup failed", "EngineDriver");
    }

    if (!rightEngine.setUpEngine(
        RIGHT_ENGINE_FRONT_PIN, 
        RIGHT_ENGINE_BACK_PIN,
        RIGHT_ENGINE_PWM_PIN, 
        RIGHT_ENGINE_PWM_CHANNEL
    ))
    {
        ErrorHandler::throwError("right engine setup failed", "EngineDriver");
    }

    leftEngine.setupEncoder(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN);
    rightEngine.setupEncoder(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN);

    controller.begin(&irSensors, &giroscope, &leftEngine, &rightEngine);

    #if defined(BLUETOOTH_ENABLE) && !defined(SIMULATION)
    BluetoothHandler::init(&controller, &irSensors);
    #endif
}

#ifdef SIMULATION
bool hasWallFront()
{
    return SIMULATOR::wallFront();
}

bool hasWalLleft()
{
    return SIMULATOR::wallLeft();
}

bool hasWallRight()
{
    return SIMULATOR::wallRight();
}
#else
bool hasWallFront()
{
    return irSensors.getFrontDistance() <= WALL_DETECTION_TRESHOLD ? true : false;
}

bool hasWalLleft()
{
    return irSensors.getLeftDistance() <= WALL_DETECTION_TRESHOLD ? true : false;
}

bool hasWallRight()
{
    return irSensors.getRightDistance() <= WALL_DETECTION_TRESHOLD ? true : false;
}
#endif

typedef struct {
    bool hasWall;
    directions direction;
} sensorPack;

void readEnviroment()
{
    sensorPack sensorRead[3] = {
        {hasWallFront(), controller.getFrontDir()},
        {hasWallRight(), controller.getRightDir()},
        {hasWalLleft(),  controller.getLeftDir()}
    };

    for (const sensorPack& read : sensorRead)
    {
        if (read.hasWall)
        {
            maze.activateCellWalls(
                controller.getPosX(),
                controller.getPosY(),
                read.direction
            );
        }
    }
}

directions nextStep()
{
    short posX = controller.getPosX();
    short posY = controller.getPosY();

    Cell* currentCell = maze.getCell(posX, posY);
    if (currentCell == nullptr)
    {
        ErrorHandler::throwError("Tried to access an out of boundary cell.", "nextStep, main.cpp");
        return controller.getFrontDir();
    }
    directions front = controller.getFrontDir();

    currentCell->setExplored(true);

    unsigned short smallestDistance = currentCell->getDistance();
    directions movDir = INVALID_DIRECTION;

    for (directions dir : allDirections)
    {
        if (!currentCell->peekWall(dir))
        {
            short dx = posX + offsetX(dir);
            short dy = posY + offsetY(dir);

            Cell* neighbor = maze.getCell(dx, dy);

            if (neighbor) {
              unsigned short d = neighbor->getDistance();
              if (d < currentCell->getDistance()) {
                if (d < smallestDistance ||
                    (d == smallestDistance && dir == front)) {
                  smallestDistance = d;
                  movDir = dir;
                }
              }
            }
        }
    }

    return movDir;
}

enum RobotState {
    WAIT_FOR_START,
    EXPLORING,
    GOAL_REACHED,
    ERROR_STOP
};

RobotState state = WAIT_FOR_START;

void loop() {
    #if defined(BLUETOOTH_ENABLE) && !defined(SIMULATION)
    BluetoothHandler::processPendingCommands();
    #endif

    switch (state) {
        case WAIT_FOR_START: {
            #if defined(BLUETOOTH_ENABLE) && !defined(SIMULATION)
            if (BluetoothHandler::isIndeterminateMoving()) {
                break;
            }
            if (BluetoothHandler::checkAndClearStartRequest()) {
                state = EXPLORING;
                break;
            }
            #endif

            // Trigger start when hand is placed within 5cm of front sensor
            if (irSensors.getFrontDistance() > 0.5f && irSensors.getFrontDistance() < 5.0f) {
                delay(1000); // Give user 1s to remove hand
                state = EXPLORING;
            }
            delay(50);
            break;
        }

        case EXPLORING: {
            short x = controller.getPosX();
            short y = controller.getPosY();

            // Check for 2x2 Micromouse center (for 16x16: x in [7,8], y in [7,8])
            if ((x == 7 || x == 8) && (y == 7 || y == 8)) {
                controller.stop();
                state = GOAL_REACHED;
                break;
            }

            // 1. Sense walls at current cell
            readEnviroment();

            // 2. Recalculate floodfill distances
            maze.updateDistances();

            // 3. Determine best neighbor and execute movement
            directions movDir = nextStep();
            if (movDir == INVALID_DIRECTION) {
                controller.stop();
                state = ERROR_STOP;
                ErrorHandler::throwError("Messed up direction. Impossible to solve!", "main.cpp");
                break;
            }

            directions currentDir = controller.getFrontDir();
            controller.turnFromAtoB(currentDir, movDir);
            controller.moveCell();
            break;
        }

        case GOAL_REACHED: {
            controller.stop();
            // Voltar pro inicio ?
            delay(500);
            break;
        }

        case ERROR_STOP: {
            controller.stop();
            delay(1000);
            break;
        }
    }

    #if defined(BLUETOOTH_ENABLE) && !defined(SIMULATION)
    if (BluetoothHandler::ServerState::getConnectionState() && BluetoothHandler::timeOut()) {
        float distances[3] = {
            irSensors.getFrontDistanceMm(),
            irSensors.getRightDistanceMm(),
            irSensors.getLeftDistanceMm()
        };
        String txValue = BluetoothHandler::formatSensorDataMsg(distances, giroscope.getYaw());
        BluetoothHandler::notifyClient(txValue);
    }
    # ifdef TUNE_WALL_DISTANCE
    char buffer[50]; 

    std::snprintf(
        buffer, 
        sizeof(buffer), 
        "WALL L : %d | WALL F : %d | WALL R %d",
        hasWalLleft(), hasWallFront(), hasWallRight()
    );
    BluetoothHandler::notifyClient(BluetoothHandler::formatTxMsg(buffer, BluetoothHandler::TOP_MSG));
    #endif
    #endif
}