# CPPND: Capstone OpenCV Multi-Robot visualizer/Simulater Example

Opencv Based multi - robot large - Scale simulater include planner and robot class
when you run it will open the enviment and load the path from text file(config/1.txt ,config/2.txt) and planner will provide the path to robots to move along the enviment , This can be use to do research in fleet management of multi robots or traffic cluttering research works
blue points are pickup points / starting points ash points a drop points / end points

## Dependencies for Running Locally
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* opencv2 >= 4.5.2-dev
  * Linux  opencv2  install (https://viking-drone.com/wiki/installing-opencv-4-5-2/)
  * Mac: same deal as make - [install opencv](https://learnopencv.com/install-opencv-4-on-macos/)
  * Windows: recommend using [install opencv](https://towardsdatascience.com/install-and-configure-opencv-4-2-0-in-windows-10-vc-d132c52063a1)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./visualizer`.


## Rubric Points

* Loops, Functions, I/O
  * The project demonstrates an understanding of C++ functions and control structures.
    * visualizer.cpp - LINE - 288,391,402.
* Loops, Functions, I/O
  * The project reads data from a file and process the data, or the program writes data to a file.
    * visualizer.cpp - LINE - 242,278,331,354.
* Object Oriented Programming
  * The project uses Object Oriented Programming techniques.
    * visualizer.h - LINE - 20,60.
* Object Oriented Programming
  * Classes use appropriate access specifiers for class members.
    * visualizer.h - LINE - 22,41,62.
* Object Oriented Programming
  * Class constructors utilize member initialization lists.
    * visualizer.cpp - LINE - 10,183.
* Memory Management
  * The project makes use of references in function declarations.
    * visualizer.cpp - LINE - 404.
* Memory Management
  * The project uses destructors appropriately.
    * visualizer.cpp - LINE - 27,195,459.
* Memory Management
  * The project uses smart pointers instead of raw pointers.
    * visualizer.cpp - LINE - 389.
* Concurrency
  * The project uses multithreading.
    * visualizer.cpp - LINE - 38,206.

