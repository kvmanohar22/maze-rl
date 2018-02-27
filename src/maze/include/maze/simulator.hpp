#ifndef _MAZE_SIMULATOR_HPP_
#define _MAZE_SIMULATOR_HPP_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

namespace maze {

class Simulator {

   double vx, vy, vz;
   double wx, wy, wz;
   ros::NodeHandle *nh;
   ros::Publisher publisher;
   ros::Rate *loop_rate;

public:
   Simulator() : vz(0), wx(0), wy(0) {
      nh = new ros::NodeHandle();
      loop_rate = new ros::Rate(10);
      publisher = nh->advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
   }

   Simulator(double vx, double vy, double wz) : vx(vx), vy(vy), wz(wz) {
      nh = new ros::NodeHandle();
      loop_rate = new ros::Rate(10);
      publisher = nh->advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
   }

   ~Simulator() {
      delete nh, loop_rate;
   }

   inline double get_time() {
      return ros::Time::now().toSec();
   }

   void move_straight(double vx, double vy, double dist);
   void rotate(double wz, double theta);
   void publish_message(geometry_msgs::Twist &msg);

};

}

#endif