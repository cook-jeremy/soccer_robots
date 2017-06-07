Homework 2: Traveling Sales-turtle

In this homework, your task is to write a controller which makes the turtle visit 10 different points on the 2D plane. The x and y coordinates of those points are defined in turtle_constants.h. The problem of visiting a set of points in a way that minimizes the distance of the entire path is called "The Traveling Salesman" problem:
https://en.wikipedia.org/wiki/Travelling_salesman_problem

Extract this package into your workspace's src folder. Rename the package to "cs309_hw2_<your eid>". Note that simply renaming the folder containing the package files is not enough to properly rename the package and may result in failure to compile it. 

To launch the environment, you'll need three terminals: in the first, start roscore. In the second, start the turtle simulation using "rosrun turtlesim turtlesim_node". In the third, start the evaluator using "rosrun cs309_homework2_<your eid> turtle_graph_traverse_evaluator". The evaluator node will spawn 10 additional turtles, each corresponding to one of the locations that needs to be visited. 

You can now start the turtle teleop node in a fourth terminal and solve the problem by controlling the turtle. Note that the evaluator node will print errors and warnings as the teleop node moves the turtle at a fairly fast speed. The speed limits your solution needs to follow are defined in turtle_constants.h. The evaluator node will detect when the turtle starts moving and keep track of any visited way-points. Once all way-points are visited, it will print out the duration and travel distance of your solution. You should restart all nodes (i.e., ctrl-c them and then run then again) each time you test, otherwise the evaluator node will keep adding more turtles in the same coordinates your extra credit solution may not be visible. 

Your solution should go in turtle_graph_traverse_hw2.cpp. To test it, start the environment as outlined in the previous paragraph and then in another terminal use rosrun to start your turtle controller node. 

There will be prizes for the solutions that visit all way points in the least amount of time! Note that your solution will be tested with a different set of coordinates from the ones included in turtle_constants.h. You are welcome to modify the coordinates to check if your solution still works -- however, do not change anything else in the header file (e.g., the speed limits!) and also do not add any functions, additional constants, etc.

EXTRA CREDIT:

1. Upon reaching the i^th way-point, your solution should use the /kill service to make the i^th turtle disappear from the 2D plane. 

2. Add a launch folder to the package and write a ROS launch file which starts the simulator and the evaluator node. Find out how to make sure the evaluator node still prints text to the console when the turtle finishes visiting all way points. 
