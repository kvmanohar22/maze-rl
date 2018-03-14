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
cv::Scalar light_green(0, 255, 128);
cv::Scalar light_red(51, 51, 255);
cv::Scalar light_blue(255, 255, 0);
cv::Scalar light_gray(96, 96, 96);

double epsilon = 0.0001;
long update_time = 1e5;

static const int rows = 10;
static const int cols = 10;


void line(cv::Mat img, cv::Point p1, cv::Point p2,
          const cv::Scalar color) {

   cv::line(img, p1, p2, green);
}


void arrow(cv::Mat &img, int direction,
           int x, int y) {
   x = x * 80 + 5 + 40;
   y = y * 80 + 5 + 40;
   cv::Point start(x, y);

   switch(direction) {
      case 0:
         cv::arrowedLine(img, start, cv::Point(x, y-40),
            utils::red, 0.3);
         break;
      case 1:
         cv::arrowedLine(img, start, cv::Point(x+40, y),
            utils::red, 0.3);
         break;
      case 2:
         cv::arrowedLine(img, start, cv::Point(x, y+40),
            utils::red, 0.3);
         break;
      case 3:
         cv::arrowedLine(img, start, cv::Point(x-40, y),
            utils::red, 0.3);
         break;
   }

}

void text(cv::Mat &img, const cv::String &text, int x, int y) {

   cv::putText(img, text, cv::Point(10+x*80, y*80+80-5),
               cv::FONT_HERSHEY_SIMPLEX,
               0.4, utils::white, 1);
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


void reset_patch(cv::Mat &img, cv::Point p1, cv::Point p2, int type=0) {
   for (int i = p1.x*80+5+1; i < p2.x*80+5; ++i)
      for (int j = p1.y*80+5+1; j < p2.y*80+5; ++j)
         if (type == 1)
            img.at<cv::Vec3b>(i, j) = cv::Vec3b(224, 224, 224);
         else if (type == 2)
            img.at<cv::Vec3b>(i, j) = cv::Vec3b(96, 96, 96);
         else
            img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
}


bool is_equal(double d1, double d2) {
    if (fabs(d1-d2) < 1e-5) {
      return true;
    } else {
      return false;
    }
}


bool is_equal(double d1, double d2, double d3) {
    if ((fabs(d1-d2) < 1e-5) && (fabs(d1-d3) < 1e-5)) {
      return true;
    } else {
      return false;
    }
}


bool is_equal(double d1, double d2, double d3, double d4) {
    if ((fabs(d1-d2) < 1e-5) && (fabs(d1-d3) < 1e-5) &&
        (fabs(d1-d4) < 1e-5)) {
      return true;
    } else {
      return false;
    }
}

bool is_corner(int r_idx, int c_idx) {
  if ((r_idx == 0 && c_idx == 0) ||
      (r_idx == 0 && c_idx == rows-1) ||
      (r_idx == cols-1 && c_idx == 0) ||
      (r_idx == cols-1 && c_idx == rows-1))
    return true;
  return false;
}

bool is_edge(int r_idx, int c_idx) {
  if (is_corner(r_idx, c_idx)) {
    return false;
  } else {
    if (r_idx == 0 || r_idx == cols-1 ||
        c_idx == 0 || c_idx == rows-1)
      return true;
    return false;
  }
}

int corner_type(int r_idx, int c_idx) {
  if (r_idx == 0 && c_idx == 0)
     return 0;
  else if (r_idx == 0 && c_idx == rows-1)
     return 3;
  else if (r_idx == cols-1 && c_idx == 0)
     return 1;
  else
     return 2;
}

int edge_type(int r_idx, int c_idx) {
  if (r_idx == 0)
     return 3;
  else if (r_idx == rows-1)
     return 1;
  else if (c_idx == 0)
     return 0;
  else
     return 2;
}

   void draw_arrows(cv::Mat img, double *vs, int i, int j) {
      int direction = -1;
      if (is_corner(i, j)) {
         switch(corner_type(i, j)) {
            case 0:
               if (utils::is_equal(vs[i*cols+j+1],
                                   vs[(i+1)*cols+j])) {
                  utils::arrow(img, 1, i, j);
                  utils::arrow(img, 2, i, j);
               } else if (vs[i*cols+j+1] > vs[(i+1)*cols+j]) {
                  utils::arrow(img, 2, i, j);
               } else {
                  utils::arrow(img, 1, i, j);
               }
               break;
            case 1:
               if (utils::is_equal(vs[(i-1)*cols+j],
                                   vs[i*cols+j+1])) {
                  utils::arrow(img, 3, i, j);
                  utils::arrow(img, 2, i, j);
               } else if (vs[(i-1)*cols+j] > vs[i*cols+j+1]) {
                  utils::arrow(img, 3, i, j);
               } else {
                  utils::arrow(img, 2, i, j);
               }
               break;
            case 2:
               if (utils::is_equal(vs[(i-1)*cols+j],
                                   vs[i*cols+j-1])) {
                  utils::arrow(img, 0, i, j);
                  utils::arrow(img, 3, i, j);
               } else if (vs[(i-1)*cols+j] > vs[i*cols+j-1]) {
                  utils::arrow(img, 3, i, j);
               } else {
                  utils::arrow(img, 0, i, j);
               }
               break;
            case 3:
               if (utils::is_equal(vs[i*cols+j-1],
                                   vs[(i+1)*cols+j])) {
                  utils::arrow(img, 0, i, j);
                  utils::arrow(img, 1, i, j);
               } else if (vs[i*cols+j-1] > vs[(i+1)*cols+j]) {
                  utils::arrow(img, 0, i, j);
               } else {
                  utils::arrow(img, 1, i, j);
               }
               break;
         }
      } else if (is_edge(i, j)) {
         switch(edge_type(i, j)) {
            case 0:
               if (utils::is_equal(vs[(i-1)*cols+j], vs[(i+1)*cols+j], vs[i*cols+j+1])) {
                  utils::arrow(img, 1, i, j);
                  utils::arrow(img, 2, i, j);
                  utils::arrow(img, 3, i, j);
               } else if (utils::is_equal(vs[(i-1)*cols+j], vs[(i+1)*cols+j])) {
                  utils::arrow(img, 1, i, j);
                  utils::arrow(img, 3, i, j);
               } else if (utils::is_equal(vs[(i-1)*cols+j], vs[i*cols+j+1])) {
                  utils::arrow(img, 2, i, j);
                  utils::arrow(img, 3, i, j);
               } else if (utils::is_equal(vs[(i+1)*cols+j], vs[i*cols+j+1])) {
                  utils::arrow(img, 1, i, j);
                  utils::arrow(img, 2, i, j);
               } else {
                  int direc = 1;
                  double max_v = vs[(i+1)*cols+j];
                  if (max_v < vs[(i-1)*cols+j]) {
                     max_v = vs[(i-1)*cols+j];
                     direc = 3;
                  }
                  if (max_v < vs[i*cols+j+1]) {
                     direc = 2;
                  }
                  utils::arrow(img, direc, i, j);
               }
               break;
            case 1:
               if (utils::is_equal(vs[(i-1)*cols+j], vs[i*cols+j-1], vs[i*cols+j+1])) {
                  utils::arrow(img, 0, i, j);
                  utils::arrow(img, 2, i, j);
                  utils::arrow(img, 3, i, j);
               } else if (utils::is_equal(vs[(i-1)*cols+j], vs[i*cols+j-1])) {
                  utils::arrow(img, 0, i, j);
                  utils::arrow(img, 3, i, j);
               } else if (utils::is_equal(vs[(i-1)*cols+j], vs[i*cols+j+1])) {
                  utils::arrow(img, 2, i, j);
                  utils::arrow(img, 3, i, j);
               } else if (utils::is_equal(vs[i*cols+j-1], vs[i*cols+j+1])) {
                  utils::arrow(img, 0, i, j);
                  utils::arrow(img, 2, i, j);
               } else {
                  int direc = 0;
                  double max_v = vs[i*cols+j-1];
                  if (max_v < vs[(i-1)*cols+j]) {
                     max_v = vs[(i-1)*cols+j];
                     direc = 3;
                  }
                  if (max_v < vs[i*cols+j+1]) {
                     direc = 2;
                  }
                  utils::arrow(img, direc, i, j);
               }
               break;
            case 2:
               if (utils::is_equal(vs[(i-1)*cols+j], vs[(i+1)*cols+j], vs[i*cols+j-1])) {
                  utils::arrow(img, 1, i, j);
                  utils::arrow(img, 0, i, j);
                  utils::arrow(img, 3, i, j);
               } else if (utils::is_equal(vs[(i-1)*cols+j], vs[(i+1)*cols+j])) {
                  utils::arrow(img, 1, i, j);
                  utils::arrow(img, 3, i, j);
               } else if (utils::is_equal(vs[(i-1)*cols+j], vs[i*cols+j-1])) {
                  utils::arrow(img, 0, i, j);
                  utils::arrow(img, 3, i, j);
               } else if (utils::is_equal(vs[(i+1)*cols+j], vs[i*cols+j-1])) {
                  utils::arrow(img, 1, i, j);
                  utils::arrow(img, 0, i, j);
               } else {
                  int direc = 1;
                  double max_v = vs[(i+1)*cols+j];
                  if (max_v < vs[(i-1)*cols+j]) {
                     max_v = vs[(i-1)*cols+j];
                     direc = 3;
                  }
                  if (max_v < vs[i*cols+j-1]) {
                     direc = 0;
                  }
                  utils::arrow(img, direc, i, j);
               }
               break;
            case 3:
               if (utils::is_equal(vs[i*cols+j-1], vs[(i+1)*cols+j], vs[i*cols+j+1])) {
                  utils::arrow(img, 1, i, j);
                  utils::arrow(img, 2, i, j);
                  utils::arrow(img, 0, i, j);
               } else if (utils::is_equal(vs[i*cols+j-1], vs[(i+1)*cols+j])) {
                  utils::arrow(img, 1, i, j);
                  utils::arrow(img, 0, i, j);
               } else if (utils::is_equal(vs[i*cols+j-1], vs[i*cols+j+1])) {
                  utils::arrow(img, 2, i, j);
                  utils::arrow(img, 0, i, j);
               } else if (utils::is_equal(vs[(i+1)*cols+j], vs[i*cols+j+1])) {
                  utils::arrow(img, 1, i, j);
                  utils::arrow(img, 2, i, j);
               } else {
                  int direc = 1;
                  double max_v = vs[(i+1)*cols+j];
                  if (max_v < vs[i*cols+j-1]) {
                     max_v = vs[i*cols+j-1];
                     direc = 0;
                  }
                  if (max_v < vs[i*cols+j+1]) {
                     direc = 2;
                  }
                  utils::arrow(img, direc, i, j);
               }
               break;
         }
      } else {
         if (utils::is_equal(vs[i*cols+j-1], vs[(i+1)*cols+j],
                             vs[i*cols+j+1], vs[(i-1)*cols+j])) {
            utils::arrow(img, 0, i, j);
            utils::arrow(img, 1, i, j);
            utils::arrow(img, 2, i, j);
            utils::arrow(img, 3, i, j);
         } else if (utils::is_equal(vs[i*cols+j-1], vs[(i+1)*cols+j], vs[i*cols+j+1])) {
            utils::arrow(img, 0, i, j);
            utils::arrow(img, 1, i, j);
            utils::arrow(img, 2, i, j);
         } else if (utils::is_equal(vs[i*cols+j-1], vs[(i+1)*cols+j], vs[(i-1)*cols+j])) {
            utils::arrow(img, 0, i, j);
            utils::arrow(img, 1, i, j);
            utils::arrow(img, 3, i, j);
         } else if (utils::is_equal(vs[i*cols+j-1], vs[i*cols+j+1], vs[(i-1)*cols+j])) {
            utils::arrow(img, 0, i, j);
            utils::arrow(img, 2, i, j);
            utils::arrow(img, 3, i, j);
         } else if (utils::is_equal(vs[(i+1)*cols+j], vs[i*cols+j+1], vs[(i-1)*cols+j])) {
            utils::arrow(img, 1, i, j);
            utils::arrow(img, 2, i, j);
            utils::arrow(img, 3, i, j);
         } else {
            double max_v = vs[i*cols+j-1];
            int direc = 0;
            if (vs[(i+1)*cols+j] > max_v) {
               max_v = vs[(i+1)*cols+j];
               direc = 1;
            } 
            if (vs[i*cols+j+1] > max_v) {
               max_v = vs[i*cols+j+1];
               direc = 2;
            }
            if (vs[(i-1)*cols+j] > max_v) {
               direc = 3;
            }
            utils::arrow(img, direc, i, j);
         }
      }
   }

}

#endif