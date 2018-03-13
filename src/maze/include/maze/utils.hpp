#ifndef _MAZE_UTILS_HPP_
#define _MAZE_UTILS_HPP_

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace utils {


// Some colors
cv::Scalar white(255, 255, 255);
cv::Scalar black(0, 0, 0);
cv::Scalar blue(255, 0, 0);
cv::Scalar green(0, 255, 0);
cv::Scalar red(0, 0, 255);


void line(cv::Mat img, cv::Point p1, cv::Point p2,
          const cv::Scalar color) {

   cv::line(img, p1, p2, color);
}


void arrow(cv::Mat &img, int direction,
           int x, int y) {

   x = x * 80 + 5 + 40;
   y = y * 80 + 5 + 40;
   cv::Point start(x, y);

   switch(direction) {
      case 0:
         cv::arrowedLine(img, start, cv::Point(x, y-40),
            utils::green, 0.3);
         break;
      case 1:
         cv::arrowedLine(img, start, cv::Point(x+40, y),
            utils::green, 0.3);
         break;
      case 2:
         cv::arrowedLine(img, start, cv::Point(x, y+40),
            utils::green, 0.3);
         break;
      case 3:
         cv::arrowedLine(img, start, cv::Point(x-40, y),
            utils::green, 0.3);
         break;
   }

}

void text(cv::Mat &img, const cv::String &text, int x, int y) {

   cv::putText(img, text, cv::Point(10+x*80, y*80+80-5),
               cv::FONT_HERSHEY_SIMPLEX,
               0.4, utils::blue, 1);
}

void construct_maze(cv::Mat &img) {
   for (int i = 0; i < 11; ++i) {
      line(img, cv::Point(5, 80*i+5), cv::Point(805, 80*i+5),
           utils::white);
   }
   for (int i = 0; i < 11; ++i) {
      line(img, cv::Point(80*i+5, 5), cv::Point(80*i+5, 805),
           utils::white);
   }
}


void reset_patch(cv::Mat &img, cv::Point p1, cv::Point p2) {
   for (int i = p1.x*80+5+1; i < p2.x*80+5; ++i)
      for (int j = p1.y*80+5+1; j < p2.y*80+5; ++j)
         img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
}


}

#endif