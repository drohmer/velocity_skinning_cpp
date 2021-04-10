# Velocity Skinning Code

- The actual implementation of velocity skinning is in the file scenes/sources/squashy_skinning/skinning.cpp

## Compilation on Windows system with Visual Studio 

- Use CMakeLists.txt with Visual Studio
- Precompiled version of GLFW3 is provided (precompiled/glfw3_win)

The executable directory need to be set to the root of the project
(Alternatively, you can copy assets/ and scenes/ directories in the executable directory)

## Compilation in command line using the Makefile (Linux/MacOS only)

$ make

$ ./scene


## Setup compilation in command line using CMake (Linux/MacOs)

This step create the build directory, call CMake to generate the Makefile, compile and execute the code. Note that the creation of the build directory and CMake call has to be done only once on a given system.

The following command assume you have opened a command line in the directory vcl/

### Create a build directory

$ mkdir build

$ cd build

### Execute CMake, compile

$ cmake ..

$ make

$ cd ..

### Execute

$ build/pgm



