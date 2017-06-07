#include "grid/grid.h"
#include <vector>
#include <stdio.h>
#include <cstdlib>
#include <ctime>
/*
* Driver file for the simulated GRID project.
* Here is where all agents will be assembled into a vector and passed into the created grid object.
*
*/

#include <agent/RandomAgent.h>

using namespace bwi_gridworld;

int main(int argc, char *argv[]){
    std::srand(time(0)); //seeds random number generator with the current time

    Grid grid(new RandomAgent());
    grid.runExperiments();
    return 0;
}
