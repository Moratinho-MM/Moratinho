#include "Simulator.h"

bool awaitResponse()
{
    unsigned long timeout_start = millis();
    while (millis() - timeout_start < 1000)
    {
        if (BluetoothHandler::hasRecivedRespone())
        {
            return BluetoothHandler::getLastResponse();
            
        }
    }
    ErrorHandler::throwError("Response timed out!", "Simulation : awaitResponse");
    return false;
}

int SIMULATOR::mazeWidth() 
{
    
}

int SIMULATOR::mazeHeight()
{
    
}

bool SIMULATOR::wallFront()
{
    BluetoothHandler::notifyClient(BluetoothHandler::formatTxMsg(".", BluetoothHandler::F_WALL_REQ));
    return awaitResponse();
}

bool SIMULATOR::wallRight()
{
    BluetoothHandler::notifyClient(BluetoothHandler::formatTxMsg(".", BluetoothHandler::R_WALL_REQ));
    return awaitResponse();

}

bool SIMULATOR::wallLeft()
{
    BluetoothHandler::notifyClient(BluetoothHandler::formatTxMsg(".", BluetoothHandler::L_WALL_REQ));
    return awaitResponse();

}

void SIMULATOR::moveForward()
{
    BluetoothHandler::notifyClient(BluetoothHandler::formatTxMsg(".", BluetoothHandler::MOV_F_REQ));
}

void SIMULATOR::turnRight()
{
    BluetoothHandler::notifyClient(BluetoothHandler::formatTxMsg(".", BluetoothHandler::TURN_R_REQ));
}

void SIMULATOR::turnLeft()
{
    BluetoothHandler::notifyClient(BluetoothHandler::formatTxMsg(".", BluetoothHandler::TURN_L_REQ)); 
}

void SIMULATOR::setWall(int x, int y, char direction)
{
   
}

void SIMULATOR::clearWall(int x, int y, char direction)
{
    
}

void SIMULATOR::setDistance(int x, int y, const int d)
{
    char buffer[5];
        std::snprintf(buffer, sizeof(buffer),
        "%d",
        d
    );
    BluetoothHandler::notifyClient(BluetoothHandler::formatTxMsg(buffer, BluetoothHandler::SET_CELL_D_REQ));
}

void SIMULATOR::clearDistance(int x, int y)
{
    BluetoothHandler::notifyClient(BluetoothHandler::formatTxMsg(".", BluetoothHandler::CLEAR_CELL_D_REQ));
}

void SIMULATOR::clearAllDistances() 
{
  
}
