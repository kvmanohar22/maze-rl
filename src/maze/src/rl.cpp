#include "maze/rl.hpp"

#include <climits>
#include <iostream>

namespace rl {

   double value_iteration::get_q_val(int r_idx, int c_idx, int action) const {
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

}