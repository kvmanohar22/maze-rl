#ifndef _MAZE_SIMULATOR_HPP_
#define _MAZE_SIMULATOR_HPP_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "turtlesim/Pose.h"

namespace maze {

class Simulator {

   turtlesim::Pose pose;
   ros::NodeHandle *nh;
   ros::Publisher publisher;
   ros::Subscriber subscriber;
   ros::Rate loop_rate;

public:
   Simulator() : loop_rate(10) {
      nh = new ros::NodeHandle();
      publisher = nh->advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
      subscriber = nh->subscribe("/turtle1/pose", 1, &Simulator::callback, this);
   }

   ~Simulator() {
      delete nh;
   }

   inline double get_time() {
      return ros::Time::now().toSec();
   }

   void move_straight(double vx, double vy, double dist);
   void rotate(double wz, double theta);
   void goto_point(geometry_msgs::Pose2D pose);
   void publish_message(geometry_msgs::Twist &msg);

   void callback(const turtlesim::PoseConstPtr &msg);

};

}

#endif
