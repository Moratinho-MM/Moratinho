#include "RequestsHandler.h"
#include "../BluetoothHandler/BluetoothHandler.h"

namespace RequestHandler {
    void ParseRequest(char* request) {
        if (request != nullptr) {
            BluetoothHandler::handleIncomingJson(std::string(request));
        }
    }
}
