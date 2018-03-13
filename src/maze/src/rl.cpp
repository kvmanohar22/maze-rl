#include "maze/rl.hpp"
#include "maze/utils.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <climits>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <math.h>

using namespace cv;
using namespace std;

namespace rl {

   void value_iteration::iterate(cv::Mat &image) {
      this->img = image;
      utils::construct_maze(img);
      namedWindow("maze", WINDOW_AUTOSIZE);
      display_vals();
      imshow("maze", img);
      waitKey(30);
      char step;
      int step_idx = 0;
      while (true) {
         cout << "Continue (y/n): ";
         // fflush(stdin);
         // fflush(stdout);
         step = getchar();
         if (step == 'n')
            exit(0);
         cout << "Step: " << step_idx+1 << endl;

         // update the value function and display the result
         single_update();
         display_vals();
         draw_arrows();

         // update the display
         imshow("maze", img);
         waitKey(1000);
         ++step_idx;
      }
   }


   double value_iteration::get_q_val(int r_idx, int c_idx, int action) const {

      if (r_idx == 7 && c_idx == 2) {
         return 3;
      }

      if (r_idx == 8 && c_idx == 7) {
         return 10;
      }

      double qval = 0.0;
      for (int dir = 0; dir < 4; ++dir) {
         double imm_reward = get_reward(r_idx, c_idx, dir);
         double prev_value = get_value(r_idx, c_idx, dir);
         if (action == dir) {
            qval += reach_prob * (imm_reward + discount * prev_value);
         } else {
            qval += non_reach_prob * (imm_reward + discount * prev_value);
         }
      }

      if (r_idx == 3 && c_idx == 7) {
         qval -= 10;
      }

      if (r_idx == 3 && c_idx == 4) {
         qval -= 5;
      }
      return qval;
   }


   double value_iteration::get_reward(int r_idx, int c_idx,
                                      int dir) const {
      switch(dir) {
         case 0:
            if (r_idx == 0)
               return -1;
            else
               return 0;
         case 1:
            if (c_idx == rows-1)
               return -1;
            else
               return 0;
         case 2:
            if (r_idx == rows-1)
               return -1;
            else
               return 0;
         case 3:
            if (c_idx == 0)
               return -1;
            else
               return 0;
         default:
            return 0;
      }
   }


   double value_iteration::get_value(int r_idx, int c_idx,
                                     int dir) const {
      switch(dir) {
         case 0:
            if (r_idx == 0)
               return value_function[r_idx * cols + c_idx];
            else
               return value_function[(r_idx-1) * cols + c_idx];
         case 1:
            if (c_idx == rows-1)
               return value_function[r_idx * cols + c_idx];
            else
               return value_function[r_idx * cols + c_idx + 1];
         case 2:
            if (r_idx == rows-1)
               return value_function[r_idx * cols + c_idx];
            else
               return value_function[(r_idx+1) * cols + c_idx];
         case 3:
            if (c_idx == 0)
               return value_function[r_idx * cols + c_idx];
            else
               return value_function[r_idx * cols + c_idx-1];
         default:
            return 0;
         }
   }


   void value_iteration::set_constants() {
      discount = 0.9;
      reach_prob = 0.7;
      non_reach_prob = 0.1;
   }


   void value_iteration::set_init_table() {
      for (int i = 0; i < rows; ++i)
         for (int j = 0; j < cols; ++j)
            value_function[i*cols+j] = 0;
   }


   void value_iteration::single_update() {
      if (!is_valid()) {
         std::cerr << "Invalid maze\n";
         exit(-1);
      }

      double *new_value_function = new double[rows*cols];
      for (int i = 0; i < rows; ++i) {
         for (int j = 0; j < cols; ++j) {
            double max_qval = double(INT_MIN);
            for (int action = 0; action < 4; ++action) {
               double curr_qval = get_q_val(i, j, action);
               if (curr_qval > max_qval) {
                  max_qval = curr_qval;
               }
            }
            new_value_function[i*cols+j] = max_qval;
         }
      }
      value_function = new_value_function;
   }


   void value_iteration::display_vals() {
      for (int i = 0; i < 10; ++i) {
         for (int j = 0; j < 10; ++j) {
            std::ostringstream txt;
            txt << roundf(value_function[i*cols+j] * 100) / 100;
            cv::String str = txt.str();
            utils::reset_patch(img, Point(j, i), Point(j+1, i+1));
            utils::text(img, str, i, j);
         }
      }
   }


   void value_iteration::draw_arrows() {
      for (int i = 0; i < 10; ++i) {
         for (int j = 0; j < 10; ++j) {
            int direction = -1;
            if (i == 0 & j == 0) {
               if (value_function[i*cols+j+1] > value_function[(i+1)*cols+j])
                  direction = 1;
               else
                  direction = 2;
            } else if (i == 0 & j == 9) {
               if (value_function[i*cols+j-1] > value_function[(i+1)*cols+j])
                  direction = 3;
               else
                  direction = 2;
            } else if (i == 9 & j == 0) {
               if (value_function[i*cols+j+1] > value_function[(i-1)*cols+j])
                  direction = 1;
               else
                  direction = 0;
            } else if (i == 9 & j == 9) {
               if (value_function[i*cols+j-1] > value_function[(i-1)*cols+j])
                  direction = 3;
               else
                  direction = 0;
            } else {
               double max_q = value_function[(i-1)*cols+j];
               direction = 0;
               if (max_q < value_function[i*cols+j+1]) {
                  direction = 1;
                  max_q = value_function[i*cols+j+1];
               }
               if (max_q < value_function[(i+1)*cols+j]) {
                  direction = 2;
                  max_q = value_function[(i+1)*cols+j];
               }
               if (max_q < value_function[i*cols+j-1]) {
                  direction = 3;
                  max_q = value_function[i*cols+j-1];
               }
            }

            // draw the direction
            utils::arrow(img, direction, i, j);
         }
      }
   }

}