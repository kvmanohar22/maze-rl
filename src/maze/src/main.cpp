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
namedWindow("maze", WINDOW_AUTOSIZE);

utils::construct_maze(image);
imshow("maze", image);

waitKey(0);

}