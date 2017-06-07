# bwi_gridworld

## Introduction

In this lab, you will work with a "Grid World" simulator. The problem is stated as follows.
Suppose you have an, n*m grid. Every few milliseconds, a "time step" goes by and some event
happens. You're given 4 agents to control. Let's call them bots. Every time step, you can
give each bot of 4 commands: north ('n'), south ('s'), west ('w') and east ('e'). Think of
this as a game of chess, where every turn certain pieces move according to some rules. At
the same time, every timestep there will be events that will pop up at completely random
locations on the grid with a probability p=0.05. Your goal is to step on as many events as
possible with the 4 bots.

## Instructions

### Creating an Agent

The first step is to create an agent. To do this, you will have to extend the Agent class
and implement your own version of the class. You can reference how to do this by looking
at the `agent/src/RandomAgent.cpp` and `include/agent/RandomAgent.h` files. The class you
will be extending can be found in `include/grid/Agent.h`. 

Notes on implementation:
* Remember to add the `.cpp` file to `agent/src/` and the `.h` file to `include/agent/`
* You will have to implement a `Agent* clone(int)` method. This will let you create different
  versions of the same agent, each with a unique ID. Since we'll be creating 4 agents,
  they will be provided the ID's 0-3. Look at the reference RandomAgent for what to return
  from the function. Only return pointers created with `new`.
* Remember to implement `char nextAction(const Pos&)`. It will be called whenever a time
  step passes and your bot needs a new action. It will provide you with the bot's current
  position. Return either 'n', 's', 'e' or 'w'.
* Remember to implement `void eventFound(const Pos&)`. It will be called if an action you
  sent caused the bot to step on an event. You can either ignore the value or use it to
  store the results.
* Your class can keep state. In other words, you can keep information on things such as:
  where you've found events, a list of actions you've taken, etc.
* If you move towards a wall, your action will be wasted.

### Running your Agent

Once you created a class with your agent, you can add it to the simulator to begin running.
All you have to do is open the `experiment/src/run_sim.cpp` file, add the line:
`#include <agent/YOUR_AGENT.h>`, where `YOUR_AGENT.h` is the name of your agent's header
file. Then replace `Grid grid(new RandomAgent());` with `Grid grid(new YOUR_AGENT());`, where
again, `YOUR_AGENT` is the name of your agent's class.

Notes on implementation:
* You can run experiments on multiple agents, one after another if you create multiple grids
  and run their `runExperiment()` method. However, remember to leave only one experiment
  running when you turn in the assignment

### Compiling and Running

The project is configured using cmake:
https://cmake.org/

Navigate to the project's root repository (where the CMakeLists.txt file). If there is no 
build folder, in the terminal run (without the $ sign):
```
$ mkdir build/
```

Then "configure" your project by running:
```
$ cd build/
$ cmake ..
```

Finally, compile and run the resulting executable:

```
$ make
$ ./gridworld
```
  
### Miscellaneous

* Do not change the `grid.cpp` file. It contains the simulator and we will use a clean
  version of the file when compiling your code. However, you are more than welcome and even
  encouraged to look at it and understand how it works.
  
## Submission Guidelines

To submit the assignment, rename the folder bwi_gridword_${UTEID}, where ${UTEID&} is your
UT EID. Put the entire solution in either a `.zip` or (preferably) a `.tar.gz` archive, and
submit it through canvas.
