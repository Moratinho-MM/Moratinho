#pragma once

#include <queue>
#include <vector>
#include "Coordinates.h"

#include <Arduino.h>

#define MAX_MAZE_SIZE 16

class Walls {
    
    /* Classe que contém 4 elementos booleanos, um para cada parede. */
    private:
    
    bool north : 1;
    bool south : 1;
    bool east  : 1;
    bool west  : 1;
    
    public:

    Walls()
    {
        north = 0;
        south = 0;
        east  = 0;
        west  = 0;
    }
    void activate(directions dir)
    {
        switch (dir)
        {
        case NORTH: north = 1; break;
        case SOUTH: south = 1; break;
        case EAST: east   = 1; break;
        case WEST: west   = 1; break;
        default: return;
        }
    }

    bool peek(directions dir) const
    {
        switch (dir)
        {
        case NORTH: return north; break;
        case SOUTH: return south; break;
        case EAST:  return east;  break;
        case WEST:  return west;  break;
        default: return false;
        }
    }

};

class Cell 
{
    private:

    unsigned short distance;
    Walls walls;
    short pos_x, pos_Y;
    bool explored;
    
    public:
    Cell(short x, short y)
    {
        pos_x = x; pos_Y = y;
        explored = false;
        distance = 0xFFFF;
    }

    void setDistance(unsigned short dis) {
        /* Seta uma distância Manhatan para a célula */
        distance = dis;
    }
    unsigned short getDistance() const {
        /* Retorna a distancia Manhaten da célula */
        return distance;
    }
    void activateWall(directions dir) {
        /* Seta a parede da direção listada como ativa */
        walls.activate(dir);
    }
    bool peekWall(directions dir) const {
        /* Retorna se a parede da direção está ativa */
        return walls.peek(dir);
    }
    void setExplored(bool explored){
        /* Seta a célula como explorada */
        this->explored = explored;
    }
    bool isExplored() const {
        /* Retorna se a célula foi explorada */
        return explored;
    }
};


class Maze 
{
    private:
    short goalX, goalY;
    unsigned short size;

    std::vector<std::vector<Cell>> grid;

    public:
    
    Maze(unsigned short size, short goal_x, short goal_y)
    {
        setGoal(goal_x, goal_y);
        
        this->size = size;

        if (this->size > MAX_MAZE_SIZE) this->size = MAX_MAZE_SIZE;

        generateMaze();
    }

    void generateMaze()
    {
        grid.clear();
        // Iniclia as linhas
        
        // Calcula distâncias desconsiderando as paredes
        for (short y = 0; y < size; y++)
        {
            std::vector<Cell> gridLine;
            for (short x = 0; x < size; x++)
            {
                gridLine.push_back(Cell(x, y));
            }
            grid.push_back(gridLine);
        }
        updateDistances();
    }

    void resetCellDistances()
    {
        for (auto& cellList : grid) // auto& é um operador de referência, em C++ os for-each loops fazem
        {                                           // cópicas de estruturas como vectors, fica a dica ;D     
            for (auto& cell : cellList)
            {
                cell.setDistance(0xFFFF);
            }
        }
    }

    void updateDistances()
    {
        resetCellDistances();
        
        std::queue<std::pair<short, short>> q;

        bool visited[MAX_MAZE_SIZE][MAX_MAZE_SIZE] = {false};

        // Inicializa a célula objetivo
        Cell* goalCell = getCell(goalX, goalY);
        if (!goalCell) return;                                      // TODO : Redirecionar erro para o ErrorHandler
        goalCell->setDistance(0);
        
        q.push({goalX, goalY});
        visited[goalY][goalX] = true;

        while (!q.empty())
        {
            auto coords = q.front();
            short x = coords.first;
            short y = coords.second;
            q.pop();

            unsigned short currentDistance = getCell(x, y)->getDistance();

            // Explorar vizinhos
            for (directions dir : allDirections) 
            {
                short dx = x + offsetX(dir);
                short dy = y + offsetY(dir);

                // Verificar limites e paredes
                if (0 <= dx && dx < size && 0 <= dy && dy < size &&
                    !getCell(x, y)->peekWall(dir) && // Sem parede nessa direção
                    visited[dy][dx] == false) // Não visitado 
                {

                    getCell(dx, dy)->setDistance(currentDistance + 1);
                    q.push({dx, dy});
                    visited[dy][dx] = true;
                }
            }
        }
    }

    std::pair<short, short> getGoal()
    {
        return {goalX, goalY};
    }

    short getGoalX() const 
    {
        return goalX;
    }

    short getGoalY() const 
    {
        return goalY;
    }

    void setGoal(short pos_x, short pos_y)
    {
        goalY = pos_y; goalX = pos_x;
    }

    unsigned short getSize()
    {
        return size;
    }

    void activateCellWalls(short x, short y, directions dir) 
    {
        Cell *c = getCell(x, y);
        if (c) c->activateWall(dir);
        else return;                                                // TODO : Redirecionar erro para o ErrorHandler
        
        short dx = x + offsetX(dir);
        short dy = y + offsetY(dir);
        
        if (0 <= dx && dx < size && 0 <= dy && dy < size)
        {
            Cell *nc = getCell(dx, dy);
            if (nc)
                nc->activateWall(opositeSide(dir));
            else return;                                            // TODO : Redirecionar erro para o ErrorHandler
        }
    }

    Cell* getCell(short x, short y) 
    {
    if (0 <= x && x < size && 0 <= y && y < size) {
        return &grid[y][x];
    }
    return nullptr; // fora dos limites
    }

};
