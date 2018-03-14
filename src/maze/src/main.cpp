#include <iostream>

#include "maze/simulator.hpp"
#include "maze/rl.hpp"
#include "maze/utils.hpp"

#include "ros/ros.h"

#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv) {

   Mat image(810, 810, CV_8UC3, Scalar(0,0,0));

   cv::Point pt;

   // currently 10, 10 is hardcoded and so is (810, 810) image dimensions
   rl::value_iteration iter(10, 10);

   // set obstacles and target points
   iter.set_obstacles({Point(3, 4), Point(3, 7)});
   iter.set_target(Point(8, 7));

   // solve the maze
   iter.iterate(image);
}