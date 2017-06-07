cmake_Minimum_required(VERSION 2.6)
project(gridworld)

include_directories(include/)

file(GLOB_RECURSE AGENT_SOURCES "agent/src/*.cpp")
file(GLOB_RECURSE EXPERIMENT_SOURCES "experiment/src/*.cpp")
file(GLOB_RECURSE GRID_SOURCES "grid/src/*.cpp")

add_executable(${PROJECT_NAME} ${AGENT_SOURCES} ${EXPERIMENT_SOURCES} ${GRID_SOURCES})