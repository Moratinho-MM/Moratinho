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

    currentCell->setExplored(true);

    // Tenta rodar o algorítmo duas vezes, caso entre em um beco sem saída.
    // Se mesmo assim não achar saída, é pq o robô está preso, o labirinto é impossível.
    for (int attempt = 0; attempt < 2; attempt++)
    {
        unsigned short smallestDistance = currentCell->getDistance();
        directions movDir = controller.getFrontDir();
        bool foundBetterPath = false;

        for (directions dir : allDirections)
        {
            if (!currentCell->peekWall(dir))
            {
                short dx = posX + offsetX(dir);
                short dy = posY + offsetY(dir);

                Cell* neighbor = maze.getCell(dx, dy);
                if (neighbor != nullptr)
                {
                    unsigned short nextDistance = neighbor->getDistance();
                    if (nextDistance < smallestDistance)
                    {
                        smallestDistance = nextDistance;
                        movDir = dir;
                        foundBetterPath = true;
                    }
                }
            }
        }

        if (foundBetterPath)
        {
            return movDir;
        }

        readEnviroment();
        maze.updateDistances();
    }

    ErrorHandler::throwWarning("No open path to goal found!", "nextStep, main.cpp");
    return controller.getFrontDir();
}
void doNextMove()
{
    
    directions movDir = nextStep();  

    directions currentDir = controller.getFrontDir();
    controller.turnFromAtoB(currentDir, movDir);

    controller.moveCell();

}

bool start = false;
void loop()
{
    while (!start) 
    {
        if (irSensors.getFrontDistance() < 5)
        {
            start = true;
            giroscope.IMU_CONFIG(); // Adicionei isto aqui para calibrar o robô depois de perfeitamente posicionado
            delay(50);
            break;
        }
        delay(200);
    }

    giroscope.IMU_READ();
    if (!(controller.getPosX() == maze.getGoalX() && controller.getPosY() == maze.getGoalY()))
    {
        readEnviroment();
        maze.updateDistances();
        doNextMove();
    }
    
    #pragma region bluetoothControll
    #ifdef BLUETOOTH_ENABLE
    #ifndef SIMULATION
    if (BluetoothHandler::ServerState::getConnectionState())
    {
        if (BluetoothHandler::timeOut())
        {
            // Monta uma mensagem contínua (ex: tempo desde que o ESP ligou)
            giroscope.IMU_READ();

            float yaw = giroscope.getYaw();

            float distances[3] = {
                irSensors.getFrontDistance(),
                irSensors.getRightDistance(),
                irSensors.getLeftDistance()
            };

            String txValue = BluetoothHandler::formatSensorDataMsg(distances, yaw);

            // Envia a notificação para o Python
            BluetoothHandler::notifyClient(txValue);
        }
    }
    #pragma endregion
    #endif
    #endif
}