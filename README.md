# RCL
RCL (Range-only Collaborative localization) is a real-time localization system for a team of ground vehicles. Each vehicle is equipped with an UWB module and wheel encoder. Three-dof poses (x,y and orientation) of all vehicles are estimated.

## Required Dependencies
- C++11
- Build essential: `sudo apt-get install build-essential`
- CMake: `sudo apt-get install cmake`
- Boost: `sudo apt-get install libboost-dev`
  
if you build the apps:
- matplotlib-cpp: `sudo apt-get install python-matplotlib python-numpy python2.7-dev`

## Building
RCL uses the CMake pre-build tool.

```
git clone https://github.com/sqn175/range_only_CL.git
cd range_only_CL
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
```

## System requirement
The system is required to be Little Endian. The program is tested on Ubuntu 16.04.