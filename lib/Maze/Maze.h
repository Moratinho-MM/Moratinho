#pragma once

#include "Coordinates.h"
#include "../ErrorHandler/ErrorHandler.h"

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
    Cell() = default;

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

// Estrutura leve para armazenar as coordenadas na stack
struct Coord {
    short x;
    short y;
};

class Maze 
{
    private:
    short goalX, goalY;
    unsigned short size;

    Cell grid[MAX_MAZE_SIZE][MAX_MAZE_SIZE];
    Coord queue[MAX_MAZE_SIZE * MAX_MAZE_SIZE];

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
        // Não há necessidade de clear() ou push_back(), apenas sobrescrevemos as células ativas
        for (short y = 0; y < size; y++)
        {
            for (short x = 0; x < size; x++)
            {
                grid[y][x] = Cell(x, y);
                if (x == 0)        grid[y][x].activateWall(WEST);
                if (x == size - 1) grid[y][x].activateWall(EAST);
                if (y == 0)        grid[y][x].activateWall(SOUTH);
                if (y == size - 1) grid[y][x].activateWall(NORTH);
            }
        }

        grid[0][0].activateWall(EAST);
        grid[0][1].activateWall(WEST);
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

    void updateDistances() {
      resetCellDistances();
      
      int head = 0; // Índice de leitura (front/pop)
      int tail = 0; // Índice de escrita (push)

      bool visited[MAX_MAZE_SIZE][MAX_MAZE_SIZE] = {false};

      // Inicializa a célula objetivo
      Cell *goalCell = getCell(goalX, goalY);
      if (!goalCell)
        ErrorHandler::throwError("Could not star a goal cell",
                                 "Maze: updateDistances");

      short goals[4][2] = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
      for (auto &g : goals) {
        Cell *c = getCell(g[0], g[1]);
        if (c) {
          c->setDistance(0);
          queue[tail++] = {g[0], g[1]};
          visited[g[1]][g[0]] = true;
        }
      }

      visited[goalY][goalX] = true;

      // Equivalente a while (!q.empty())
      while (head < tail) {
        // Equivalente a q.front() + q.pop()
        Coord current = queue[head++];
        short x = current.x;
        short y = current.y;

        Cell *currentCell = getCell(x, y);
        unsigned short currentDistance = currentCell->getDistance();

        // Explorar vizinhos
        for (directions dir : allDirections) {
          short dx = x + offsetX(dir);
          short dy = y + offsetY(dir);

          // Verificar limites, paredes e se já foi visitado
          if (0 <= dx && dx < size && 0 <= dy && dy < size &&
              !currentCell->peekWall(dir) && !visited[dy][dx]) {
            getCell(dx, dy)->setDistance(currentDistance + 1);
            queue[tail++] = {dx, dy}; // push
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
        else ErrorHandler::throwError("Tried to access out of boudry cell", "Maze: activateCellWalls");                                                // TODO : Redirecionar erro para o ErrorHandler
        
        short dx = x + offsetX(dir);
        short dy = y + offsetY(dir);
        
        if (0 <= dx && dx < size && 0 <= dy && dy < size)
        {
            Cell *nc = getCell(dx, dy);
            if (nc)
                nc->activateWall(opositeSide(dir));
            else ErrorHandler::throwError("No oposite side for this cell", "Maze: activateCellWalls");                                                // TODO : Redirecionar erro para o ErrorHandler
;                                            
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
