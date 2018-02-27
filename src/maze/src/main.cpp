#include <iostream>

#include "maze/simulator.hpp"
#include "ros/ros.h"

using namespace std;
using namespace maze;

int main(int argc, char **argv) {

   cout << "Running the simulator\n";
   ros::init(argc, argv, "simulator");
   Simulator sim;

   // sim.move_straight(1, 0, 3);
   sim.rotate(0.2, 1);
   return 0;
}