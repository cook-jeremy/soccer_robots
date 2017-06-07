#ifndef RandomAgent_h__guard
#define RandomAgent_h__guard


#include <grid/Agent.h>

struct RandomAgent : public bwi_gridworld::Agent {

    char nextAction(const bwi_gridworld::Pos &currentPos);
    void eventFound(const bwi_gridworld::Pos &currentPos);
    bwi_gridworld::Agent *clone(int id);

};


#endif
