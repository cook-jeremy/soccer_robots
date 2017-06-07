//BWI Patrolling gridworld project


#include <vector>
#include <valarray>
#include <stdio.h>
#include <iostream>
#include <grid/grid.h>
#include <grid/Agent.h>
#include <time.h>
#include <cstdlib>

#define NUM_TESTS 2000

#define MAX_STEPS 500

#define EVENT_PROB 0.05
#define EVENT_DURATION 25

/*
TODO: Add verbose mode?
Add in automated testing

GRID class
Creates a 2D grit, initializes agents in the map. At some point an event is placed on the map and the time is recorded from when the event is placed and an agent find's it.


*/

using namespace std;

namespace bwi_gridworld {

    Grid::Grid(Agent *prototype) : event_locations(), agents(), eventsCreated(0), eventsFound(0), step_count(0){

        agents.push_back(prototype->clone(0));
        agents.push_back(prototype->clone(1));
        agents.push_back(prototype->clone(2));
        agents.push_back(prototype->clone(3));

        delete prototype;

        //initialized agents at their starting locations (currently the four corners of the grid)
        //TODO: Allow choice of where to start agents?

        reset();
    }

    bool Grid::validMove(int agent_id, char direction){
        //printf("agent_positions[%d] = %x\n", agent_id, agent_positions[agent_id]);
        //printf("direct: %c\n", direction);
        if(agent_id < agents.size()){

            int agent_x = agent_positions[agent_id].x;
            int agent_y = agent_positions[agent_id].y;
            if(direction == 's' && ((agent_y - 1) < 0))
                return false;
            else if(direction == 'e' && ((agent_x) + 1 >= width))
                return false;
            else if(direction == 'n' && ((agent_y + 1) >= height))
                return false;
            else if(direction == 'w' && ((agent_x - 1) < 0))
                return false;
            else
                return true;
        }

        return false;
    }
    int Grid::step(int agent_id, char direction) {
        if(validMove(agent_id, direction) ){

            if(rand() <= (1 - agent_id * 0.2) * RAND_MAX) {

                if(direction == 'n')
                    agent_positions[agent_id].y++;
                if(direction == 'e')
                    agent_positions[agent_id].x++;
                if(direction == 's')
                    agent_positions[agent_id].y--;
                if(direction == 'w')
                    agent_positions[agent_id].x--;
            }
        }
        else
            std::cout << "Invalid movement! " << std::endl;
    }

    //probablistically generates events and places them on the grid
    int Grid::eventInit(){

        if(std::rand()  <= EVENT_PROB * RAND_MAX){

            int random_x, random_y;
            do {
                random_x = std::rand() % width;
                random_y = std::rand() % height;
            } while(alreadyOccupied(random_x, random_y));

            event_locations.push_back(Pos(random_x, random_y, step_count));
            eventsCreated++;
            //std::cout << "Event at location: " << random_x << ", " << random_y << std::endl;
        }

        //clear old events
        vector<Pos>::iterator evt = event_locations.begin();
        while(evt != event_locations.end()) {
            if(evt->time <= (step_count - EVENT_DURATION))
                evt = event_locations.erase(evt);
            else
                ++evt;
        }

    }

    //checks if an EVENT is already occupying a grid
    //helper method.
    bool Grid::alreadyOccupied(int x, int y){
        for(int i = 0; i < event_locations.size(); i++){
            if(event_locations.at(i).x == x && event_locations.at(i).y == y)
                return true;
        }
        return false;
    }

    int Grid::printResults(){
        std::cout << "Number of intruders found in this experiment: " << eventsFound << "." << std::endl;
        return eventsFound;
    }

    void Grid::checkIfEventFound(int agent_id){

        vector<Pos>::iterator evt = event_locations.begin();
        while(evt != event_locations.end()) {
            if(agent_positions[agent_id] ==  *evt){
                eventsFound++;
                //removes the event from the event vector
                evt = event_locations.erase(evt);

                //notifies agent
                agents[agent_id]->eventFound(agent_positions[agent_id]);
            }
            else
                ++evt;
        }
    }

    //Polls the agents for their next move
    void Grid::next() {
        eventInit();
        //First, check if the round over


        for(int i = 0; i < agents.size(); i++) {
            char agent_action = agents.at(i)->nextAction(agent_positions[i]);
            if(validMove(i, agent_action)) {
                step(i, agent_action);
            }

            checkIfEventFound(i);
        }
    }
    void Grid::runExperiments() {

        valarray<double> fractions(NUM_TESTS);


        for(int i = 0; i < NUM_TESTS; i++) {
            for(step_count = 0; step_count < MAX_STEPS; ++step_count) {
                next();
            }

            fractions[i] = eventsFound / ((double) eventsCreated);
            reset();
        }

        //computing confidence interval
        double sampleMean = fractions.sum() / NUM_TESTS;

        valarray<double> smean(sampleMean,NUM_TESTS);

        valarray<double> difference = fractions - smean;

        double std = sqrt( (difference * difference).sum() / (NUM_TESTS - 1));

        double confidence = 1.960 * (std / sqrt(NUM_TESTS));

        //std::cout << "The experiments are over. Your agents found " << eventsFound << " events, out of " << eventsCreated << "." << std::endl;
        std::cout << "Your agents found " << (sampleMean * 100) << "% +- " << (100 * confidence) << " of the threats" << std::endl;

    }

    void Grid::reset() {
        event_locations.clear();
        eventsFound =0;
        eventsCreated =0;

        //initialized agents at their starting locations (currently the four corners of the grid)
        //TODO: Allow choice of where to start agents?

        agent_positions.clear();
        agent_positions.push_back(Pos(0,0,0));
        agent_positions.push_back(Pos(width-1, 0,0));
        agent_positions.push_back(Pos(width-1, height-1,0));
        agent_positions.push_back(Pos(0, height-1,0));
    }

    Grid::~Grid() {
        vector<Agent*>::iterator it = agents.begin();
        for(; it!= agents.end(); ++it)
            delete *it;
    }


    //methods to return information to Agents
    const int Grid::getWidth(){return width;}
    const int Grid::getHeight(){return height;}


    Pos::Pos(int x, int y, int time) : x(x), y(y), time(time){};
    Pos::Pos() : x(0), y(0), time(0) {}

    bool Pos::operator==(const Pos& pos) {
        return this->x == pos.x && this->y == pos.y;
    }


}
