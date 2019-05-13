# RCL
RCL (Range-only Collaborative localization) is a real-time localization system for a team of ground vehicles. Each vehicle is equipped with an UWB module and wheel encoder. Three-dof poses (x,y and orientation) of all vehicles are estimated.

## Required Dependencies
- C++11
- Build essential: `sudo apt-get install build-essential`
- CMake: `sudo apt-get install cmake`
- Boost: `sudo apt-get install libboost-dev`
  
Optional (if you build the app with GUI):
- OpenGL (Desktop / ES / ES2)
- Glew: `sudo apt-get install libglew-dev`

## System requirement
The system is required to be Little Endian. The program is tested on Ubuntu 16.04.