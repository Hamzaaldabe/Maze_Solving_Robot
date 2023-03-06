#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "API.h"
#define MAZE_SIZE 16

// Struct to represent a square in the maze
typedef struct Square {
    int x;
    int y;
    bool visited;
    int distance;
    bool top;
    bool down;
    bool left;
    bool right;
} Square;
Square maze[MAZE_SIZE][MAZE_SIZE];

// Function to perform the flood fill algorithm to find the shortest path
void readMaze() {
  int startX = 0; // starting x position
  int startY = 0; // starting y position

  // Initialize maze squares
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      maze[x][y].x = x;
      maze[x][y].y = y;
      maze[x][y].visited = false;
      maze[x][y].distance = -1;
      maze[x][y].top = false;
      maze[x][y].down = false;
      maze[x][y].left = false;
      maze[x][y].right = false;
    }
  }
  // Initialize starting square distance to 0
  maze[startX][startY].distance = 0;
  // Turn right so that the robot is facing the right-hand wall
  //turnright();
  // Explore the maze
  int x = startX;
  int y = startY;
  int x_prev = -1;
  int y_prev = -1;
  bool finished = false;

  while (!finished) {
    //Mark current square as visited
    maze[x][y].visited = true;
    //Check for walls
    // bool rightWall = Drightwall();
   // bool leftWall  = Dleftwall();
   // bool frontWall = Dfrontwall();
    // Calculate distance for current square
    if (x_prev != -1 && y_prev != -1) {
      maze[x][y].distance = maze[x_prev][y_prev].distance + 1;
    }
    // If no wall to the right and unvisited square to the right, move right
    if (!API_wallRight() && x < MAZE_SIZE - 1 && !maze[x + 1][y].visited) {
      API_turnRight();
      x_prev = x;
      y_prev = y;
      x++;
      maze[x][y].visited = true;
      maze[x][y].left = true;
      maze[x - 1][y].right = true;
    }
    // If no wall in front and unvisited square in front, move forward
     else if (!API_wallFront() && y < MAZE_SIZE - 1 &&  !maze[x][y + 1].visited) {
      API_moveForward();
      x_prev = x;
      y_prev = y;
      y++;
      maze[x][y].visited = true;
      maze[x][y].top = true;//false***
      maze[x][y - 1].down = true;
    }
    else if (API_wallRight() && API_wallFront()) {
      API_turnLeft();
      x_prev = x;
      y_prev = y;
    }
    // If no wall to the left and unvisited square to the left, move left
    else if (!maze[x][y - 1].left && y > 0 && !maze[x][y - 1].visited) {
      API_turnLeft();
      x_prev = x;
      y_prev = y;
      y--;
      maze[x][y].visited = true;
      maze[x][y].right = true;
      maze[x][y + 1].left = true;
    }
    // If no unexplored squares to the left, turn right
    else {
      API_turnRight();
      x_prev = x;
      y_prev = y;
    }
    // Check if we've finished exploring the maze
    if (maze[MAZE_SIZE - 1][MAZE_SIZE - 1].visited) {
      finished = true;
    }
  }
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
    maze[x][y].visited = false;
    }
  }
  //solve

}


int main() {
    readMaze();
return 0;
}
