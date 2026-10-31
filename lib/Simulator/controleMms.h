#pragma once

#include "API.h"
#include "Ratao.h"
#include <string>
#include <stdio.h>
#include <iostream>
#include "Coordenadas.h"
#include "Labirinto.h"


void printDebug(std::string texto) {
    // Para escrever no simulador, deve se utilizar o buffer de erros. 
    std::cerr << texto << std::endl;
}

void escreverDistancias(Labirinto *lab) {

    unsigned int comprimento = lab->verTamanho();
    for (int y = 0; y < comprimento; y++) {
        for (int x = 0; x < comprimento; x++){
            SIMULATOR::setText(x, y, std::to_string(lab->verCelula(x, y)->getDistance()));
        }
    }

}

// função para debugar o algorítimo de Floof fill. Preenche todas as paredes existenetes.
void desenharParedes(Labirinto* lab){
    unsigned int comprimento = lab->verTamanho();
    
    for (int y = 0; y < comprimento; y++) {
        for (int x = 0; x < comprimento; x++){
            for (auto dir : allDirections){
                if (lab->verCelula(x, y)->peekWall(dir)){
                    API::setWall(x, y, translateToAPI(dir));
                }
            }
        }
    }
}

void marcarObjetivo(Labirinto* labirinto){
    SIMULATOR::setColor(labirinto->verObjetivo().first, 
    labirinto->verObjetivo().second, 'g');
}

void limparArena(Labirinto* labirinto){
    for (int y = 0; y < labirinto->verTamanho(); y++) {
        for (int x = 0; x < labirinto->verTamanho(); x++){
            if (labirinto->verCelula(x, y)->getExplored())
                SIMULATOR::setColor(x, y, 'R');
            else
                SIMULATOR::setColor(x, y, 'k');

        }
    }
}

void mostrarProximoPasso(Labirinto* labirinto, short posX, short posY){

    unsigned short distanciaAtual = labirinto->verCelula(posX, posY)->getDistance();

    // verifica para qual direção se mover
    SIMULATOR::setColor(posX, posY, 'k');
    short proximoX, proximoY;
    for (auto dir : allDirections)
    {

        short dx = posX + offsetX(dir);
        short dy = posY + offsetY(dir);

        if ((0 <= dx && dx < labirinto->verTamanho()) && (0 <= dy && dy < labirinto->verTamanho()))
        {
            if (!labirinto->verCelula(posX, posY)->peekWall(dir))
            {
                unsigned short proximaDistancia = labirinto->verCelula(dx, dy)->getDistance();
                if (proximaDistancia < distanciaAtual)
                {
                    distanciaAtual = proximaDistancia;
                    proximoX = dx;
                    proximoY = dy;
                    break;
                }
            }
        }
    }

    SIMULATOR::setColor(proximoX, proximoY, 'y');
}

void mostarMenorCaminho(Labirinto* labirinto, short origemX, short origemY, short finalX, short finalY){

    limparArena(labirinto);
    short tamanho = labirinto->verTamanho();

    // Inicializa a célula objetivo

    short proximoX = origemX;
    short proximoY = origemY;

    while (!(proximoX == finalX && proximoY == finalY))
    {
        
        SIMULATOR::setColor(proximoX, proximoY, 'b');
        
        short distanciaAtual = labirinto->verCelula(proximoX, proximoY)->getDistance();

        // Explorar vizinhos
        for (auto dir : allDirections)
        {
            short dx = proximoX + offsetX(dir);
            short dy = proximoY + offsetY(dir);

            if ((0 <= dx && dx < labirinto->verTamanho()) && (0 <= dy && dy < labirinto->verTamanho()))
            {
                if (!labirinto->verCelula(proximoX, proximoY)->peekWall(dir))
                {
                    unsigned short proximaDistancia = labirinto->verCelula(dx, dy)->getDistance();
                    if (proximaDistancia < distanciaAtual)
                    {
                        distanciaAtual = proximaDistancia;
                        proximoX = dx;
                        proximoY = dy;
                        break;         
                    }
                }
            }
        }
        marcarObjetivo(labirinto);
    }
}