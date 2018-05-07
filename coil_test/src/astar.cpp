/* Reference:
https://stackoverflow.com/questions/19193413/pathfinding-in-a-grid-system
*/

#include <bits/stdc++.h>
#include "astar.hpp"
using namespace std;

// A Utility Function to check whether given cell (row, col) is a valid cell or not.
bool isValid(int row, int col){
    return (row >= 0) && (row < ROW) && (col >= 0) && (col < COL);
}

// A Utility Function to check whether the given cell is blocked or not
bool isUnBlocked(int** grid, int row, int col){
    return grid[row][col] == 1 ? true : false;
}

// A Utility Function to check whether destination cell has been reached or not
bool isDestination(int row, int col, Pair dest){
    return row == dest.first && col == dest.second ? true : false;
}

// A Utility Function to calculate the 'h' heuristics.
double calculateHValue(int row, int col, Pair dest){
    // Return using the distance formula
    return (double)(abs(row-dest.first) + abs(col-dest.second));
}

stack<Pair> path(cell** cellDetails, Pair dest){
    int row = dest.first;
    int col = dest.second;

    stack<Pair> Path;

    while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col )){
        Path.push (make_pair (row, col));
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }

    Path.push (make_pair (row, col));
    return Path;
}

// A Utility Function to trace the path from the source to destination
void tracePath(stack<Pair> Path){
    while (!Path.empty()){
        Pair p = Path.top();
        Path.pop();
        printf("-> (%d,%d) ",p.second,p.first);
    }
    printf("\n");
}

bool searchSuccessor(cell** cellDetails, bool closedList[][COL], int** grid, set<pPair> &openList, int i, int j, int k, int l, Pair dest, stack<Pair> &Path){
    // To store the 'g', 'h' and 'f' of the 8 successors
    double gNew, hNew, fNew;

    // Only process this cell if this is a valid one
    if (isValid(k, l) == true){
        // If the destination cell is the same as the current successor
        if (isDestination(k, l, dest) == true){
            // Set the Parent of the destination cell
            cellDetails[k][l].parent_i = i;
            cellDetails[k][l].parent_j = j;
            printf ("The destination cell is found\n");
            Path = path(cellDetails, dest);
            tracePath (Path);
            return true;
        }
        // If the successor is already on the closed list or if it is blocked, then ignore it.
        else if (closedList[k][l] == false && isUnBlocked(grid, k, l) == true){
            gNew = cellDetails[i][j].g + 1.0;
            hNew = calculateHValue (k, l, dest);
            fNew = gNew + hNew;

            // If it isn’t on the open list, add it to the open list. Make the current square the parent of this square
            // If it is on the open list already, check to see if this path to that square is better
            if (cellDetails[k][l].f == FLT_MAX || cellDetails[k][l].f > fNew){
                openList.insert( make_pair(fNew, make_pair(k, l)));

                // Update the details of this cell
                cellDetails[k][l].f = fNew;
                cellDetails[k][l].g = gNew;
                cellDetails[k][l].h = hNew;
                cellDetails[k][l].parent_i = i;
                cellDetails[k][l].parent_j = j;
            }
        }
    }
    return false;
}

void printMap(int** grid){
    for(int i = 0; i < ROW; i++){
        for(int j = 0; j < COL; j++){
            printf("%d,", grid[i][j]);
        }
        printf("\n");
    }
}

void printMap_input(char maze_input[][COL]){
    for(int i = 0; i < ROW; i++){
        for(int j = 0; j < COL; j++){
            printf("%c,", maze_input[i][j]);
        }
        printf("\n");
    }
}

void convertMap(char maze_input[][COL] , int grid[][COL], Pair &src){
    // Create grid map that can be parsed by A*
    for (int i = 0; i < ROW; i++){
        for (int j = 0; j < COL; j++){
            grid[i][j] = (maze_input[i][j] == 'B' || maze_input[i][j] == 'U') ? 0 : 1;
            if (maze_input[i][j] == 'W' || maze_input[i][j] == 'A' || maze_input[i][j] == 'S' || maze_input[i][j] == 'D'){
                src = make_pair(i,j);
            }
        }
    }
}

float map_to_world(int map_coord){
    return map_coord*SCALE;
}

int world_to_map(float world_coord){
    return int(world_coord/SCALE);
}

// A* Function to find the shortest path between a given source cell to a destination cell
void aStarSearch(int** &grid, Pair src, Pair dest, stack<Pair> &Path){
    // If the source is out of range
    if (isValid (src.first, src.second) == false){
        printf ("Source is invalid\n");
        return;
    }

    // If the destination is out of range
    if (isValid (dest.first, dest.second) == false){
        printf ("Destination is invalid\n");
        return;
    }

    // Either the source or the destination is blocked
    if (isUnBlocked(grid, src.first, src.second) == false || isUnBlocked(grid, dest.first, dest.second) == false){
        printf ("Source or the destination is blocked\n");
        return;
    }

    // If the destination cell is the same as source cell
    if (isDestination(src.first, src.second, dest) == true){
        printf ("We are already at the destination\n");
        return;
    }

    // Create a closed list and initialise it to false which means that no cell has been included yet
    bool closedList[ROW][COL];
    memset(closedList, false, sizeof (closedList));
    // cell cellDetails[ROW][COL]; // Declare a 2D array of structs to hold the details of each grid cell

    cell** cellDetails = new cell*[ROW];
    for(int i=0; i<ROW; ++i){
        cellDetails[i] = new cell[COL];
    }

    // int** occ_grid = new int*[ROW];
    // for(int i = 0; i < ROW; ++i){
    //     occ_grid[i] = new int[COL];
    // }


    // Set each cost node to the maximum float value
    int i = 0; int j = 0;
    for (i=0; i<ROW; i++){
        for (j=0; j<COL; j++){
            cellDetails[i][j].f = FLT_MAX;
            cellDetails[i][j].g = FLT_MAX;
            cellDetails[i][j].h = FLT_MAX;
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
        }
    }

    // Initialize the parameters of the starting node
    i = src.first, j = src.second;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;

    set<pPair> openList; // Create an open list having information as- <f, <i, j>>, f=g+h, i, j are indices of that cell
    openList.insert(make_pair (0.0, make_pair (i, j))); // Put the starting cell on the open list and set its 'f' as 0
    bool foundDest = false; // We set this boolean value as false as initially the destination is not reached

    // Run through all elements of openList until the set is empty
    while (!openList.empty()){
        pPair p = *openList.begin();
        openList.erase(openList.begin()); // Remove this vertex from the open list

        // Add this vertex to the closed list
        i = p.second.first;
        j = p.second.second;
        closedList[i][j] = true;

        // To store the 'g', 'h' and 'f' of the 8 successors
        double gNew, hNew, fNew;

        // Assume only up, down, left, or right motions are possible
        if(searchSuccessor(cellDetails, closedList, grid, openList, i, j, i-1, j, dest, Path)) return;
        if(searchSuccessor(cellDetails, closedList, grid, openList, i, j, i+1, j, dest, Path)) return;
        if(searchSuccessor(cellDetails, closedList, grid, openList, i, j, i, j+1, dest, Path)) return;
        if(searchSuccessor(cellDetails, closedList, grid, openList, i, j, i, j-1, dest, Path)) return;
        if(searchSuccessor(cellDetails, closedList, grid, openList, i, j, i-1, j+1, dest, Path)) return;
        if(searchSuccessor(cellDetails, closedList, grid, openList, i, j, i-1, j-1, dest, Path)) return;
        if(searchSuccessor(cellDetails, closedList, grid, openList, i, j, i+1, j+1, dest, Path)) return;
        if(searchSuccessor(cellDetails, closedList, grid, openList, i, j, i+1, j+1, dest, Path)) return;
    }

    // Free memory allocated for cellDetails
    for(int i = 0; i < ROW; ++i) {
        delete [] cellDetails[i];
    }
    delete [] cellDetails;

    // When destination cell is not found and open list is empty, then we failed to reach the destination cell
    if (foundDest == false)
        printf("Failed to find the Destination Cell\n");

    return ;
}

// Test above function
// int main(){
int astar_main(){
    /* Description of the Grid-
    1--> The cell is not blocked
    0--> The cell is blocked
    NOTE: Each grid cell represents where the robot's centre could potentially be
    When the map outputs B or O, it must account for the size of the robot
        */

    stack<Pair> Path;
    int grid[ROW][COL];
    Pair src, dest;

    // Create grid map that can be parsed by A*
    // convertMap(maze_input, grid, src);
    dest = make_pair(22,25);

    int** occ_grid = new int*[ROW];
    for(int i = 0; i < ROW; ++i){
        occ_grid[i] = new int[COL];
    }

    for(int i = 0; i < ROW; ++i){
        for(int j = 0; j < COL; ++j){
            occ_grid[i][j] = 1;
        }
    }

    // printf("Source: (%d,%d)\n", src.first,src.second);
    // printf("Destination: (%d,%d)\n", dest.first,dest.second);
    printMap(occ_grid);

    // aStarSearch(grid, src, dest, Path); // Run A*
    // aStarSearch(occ_grid, src, dest, Path); // Run A*

    Pair p = Path.top();
    int WayX = p.first;
    int WayY = p.second;
    printf("(Way X,Way Y) = (%d,%d)\n", WayX, WayY);

return(0);
}
