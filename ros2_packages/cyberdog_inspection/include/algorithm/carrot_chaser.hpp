#pragma once

#include <cmath>
#include <iostream>
#include <vector>

namespace algorithm::carrot_chaser {

class CarrotChaser {
public:
  CarrotChaser(const std::vector<std::vector<float>> & path, float forward_step, float tol);
  ~CarrotChaser() = default;

  std::pair<bool, std::vector<float>> update(const std::vector<float> & current_position);
  bool is_completed() const { return completed_; }
  void reset();

private:
  float compute_distance(const std::vector<float> & a, const std::vector<float> & b);

  std::vector<std::vector<float>> path_;
  float forward_step_;
  float tol_;
  std::vector<float> current_goal_;
  int current_goal_idx_ = 0;
  bool completed_ = false;
};

}  // namespace algorithm::carrot_chaser
