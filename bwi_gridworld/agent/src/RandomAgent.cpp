#include <agent/RandomAgent.h>

#include <cstdlib>

//Max's probabilistic agent - codename 007

bwi_gridworld::Agent* RandomAgent::clone(int ) {
    return new RandomAgent(); //random agents don't use ids because they all do the same thing
}

char RandomAgent::nextAction(const bwi_gridworld::Pos& currentPos) {

    int move = std::rand() % 4;
    // std::cout << "move: " << move << std::endl;
    switch(move) {
        case 0 : return 'n'; break;
        case 1 : return 's'; break;
        case 2 : return 'e'; break;
        case 3 : return 'w'; break;
    }
}

void RandomAgent::eventFound(const bwi_gridworld::Pos &currentPos) {}
