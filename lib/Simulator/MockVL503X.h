#pragma once
#include "API.h"

class medidorDistancia {
    private:
    char orientacao;
    public:
    medidorDistancia(char direcao) {
        orientacao = direcao;
    }

    bool ler(){
        switch (orientacao)
        {
        case 'F': return SIMULATOR::wallFront(); break;
        case 'D': return SIMULATOR::wallRight(); break;
        case 'E': return SIMULATOR::wallLeft(); break;
        default: return false; break;
        }
    }
};