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
   // utils::construct_maze(image);
   // namedWindow("maze", WINDOW_AUTOSIZE);

   // utils::text(image, "23.04", 4, 5);
   // imshow("maze", image);
   // waitKey(8000);

   // utils::reset_patch(image, Point(5, 4), Point(5+1, 4+1));
   // imshow("maze", image);
   // waitKey(8000);

   // utils::text(image, "-23.04", 4, 5);
   // imshow("maze", image);
   // waitKey(8000);

   rl::value_iteration iter(10, 10);
   iter.iterate(image);
}