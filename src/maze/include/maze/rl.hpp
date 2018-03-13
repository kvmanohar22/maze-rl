#ifndef _MAZE_RL_HPP_
#define _MAZE_RL_HPP_

#include <opencv2/opencv.hpp>

namespace rl {

/**
 * @brief      VALUE ITERATION
 */

class value_iteration {

   /** Action convention
    * 0 -> up
    * 1 -> right
    * 2 -> down
    * 3 -> left
    */

   int rows, cols;
   double discount;
   double reach_prob, non_reach_prob;
   double *value_function;
   cv::Mat img;

public:
   value_iteration() : rows(-1), cols(-1) {}
   value_iteration(int rows, int cols) : rows(rows), cols(cols) {
      value_function = new double[rows*cols];
      set_constants();
      set_init_table();
   }

   ~value_iteration() {
      delete [] value_function;
   }

   double get_q_val(int r_idx, int c_idx, int action) const;
   double get_reward(int r_idx, int c_idx, int dir) const;
   double get_value(int r_idx, int c_idx, int dir) const;
   void set_constants();
   void set_init_table();
   void single_update();
   void display_vals();
   void draw_arrows();
   void iterate(cv::Mat &img);

   inline bool is_valid() const {
      return rows != -1;
   }

};

}  // namespace rl

#endif
