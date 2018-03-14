#ifndef _MAZE_RL_HPP_
#define _MAZE_RL_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

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
   double *vs;
   cv::Mat img;
   cv::Point target;
   std::vector<cv::Point> obstacles;

public:
   value_iteration() : rows(-1), cols(-1) {}
   value_iteration(int rows, int cols) : rows(rows), cols(cols) {
      vs = new double[rows*cols];
      set_constants();
      set_init_table();
   }

   ~value_iteration() {
      delete [] vs;
   }

   double get_q_val(int r_idx, int c_idx, int action) const;
   double get_reward(int r_idx, int c_idx, int dir) const;
   double get_value(int r_idx, int c_idx, int dir) const;
   void set_constants();
   void set_init_table();
   void single_update();
   void display_vals();
   void iterate(cv::Mat &img);

   // obstacles are the grids with negative reward
   void set_obstacles(std::vector<cv::Point>);
   // target is the grid with positive reward
   void set_target(cv::Point);

   inline bool is_valid() const {
      return rows != -1;
   }

   inline bool is_obstacle(cv::Point pt) {
      for (int i = 0; i < obstacles.size(); ++i)
         if (obstacles[i] == pt)
            return true;
      return false;
   }

   inline bool is_target(cv::Point pt) {
      return pt == this->target;
   }

};

}  // namespace rl

#endif
