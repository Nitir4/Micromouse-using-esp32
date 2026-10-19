cmake_minimum_required(VERSION 3.12)
project(micrmouse_simulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(micrmouse_simulator
    src/AsciiRenderer.cpp
    src/FloodFillAlgorithm.cpp
    src/Maze.cpp
    src/MazeGenerator.cpp
    src/MazeLoader.cpp
    src/RandomWalkAlgorithm.cpp
    src/RightHandRuleAlgorithm.cpp
    src/Simulator.cpp
    src/main.cpp
)

target_include_directories(micrmouse_simulator PRIVATE include)

if(MSVC)
    target_compile_options(micrmouse_simulator PRIVATE /W4)
else()
    target_compile_options(micrmouse_simulator PRIVATE -Wall -Wextra -pedantic)
endif()
