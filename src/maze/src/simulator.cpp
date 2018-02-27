#include "maze/simulator.hpp"

namespace maze {

void Simulator::move_straight(
   double vx, double vy, double dist) {
   geometry_msgs::Twist msg;
   msg.linear.x = vx;
   msg.linear.y = vy;
   msg.angular.z = 0;
   
   double begin = this->get_time();
   double dist_travelled = 0;
   while (dist_travelled < dist) {
      this->publish_message(msg);
      double current_time = this->get_time();
      double dx = vx * (current_time - begin);
      double dy = vy * (current_time - begin);
      dist_travelled = pow(pow(dx, 2) + pow(dy, 2), 0.5);
      ros::spinOnce();
      loop_rate->sleep();
   }

}

void Simulator::rotate(double wz, double theta) {
   geometry_msgs::Twist msg;
   msg.linear.x = 0;
   msg.linear.y = 0;
   msg.angular.z = wz;

   double begin = this->get_time();
   double angle_turned = 0;
   while (angle_turned < theta) {
      this->publish_message(msg);
      double current_time = this->get_time();
      angle_turned = wz * (current_time - begin);

      ros::spinOnce();
      loop_rate->sleep();
   }

}

void Simulator::publish_message(geometry_msgs::Twist &msg) {
   publisher.publish(msg);
}

}
