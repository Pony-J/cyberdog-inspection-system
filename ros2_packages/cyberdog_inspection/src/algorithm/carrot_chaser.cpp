#include "algorithm/carrot_chaser.hpp"

namespace algorithm::carrot_chaser {

CarrotChaser::CarrotChaser(
  const std::vector<std::vector<float>> & path, float forward_step, float tol)
: path_(path), forward_step_(forward_step), tol_(tol)
{
  if (!path_.empty()) {
    current_goal_ = path_[0];
  }
}

void CarrotChaser::reset()
{
  current_goal_idx_ = 0;
  completed_ = false;
  if (!path_.empty()) {
    current_goal_ = path_[0];
  } else {
    std::cerr << "[CarrotChaser] Warning: path is empty in reset()" << std::endl;
  }
}

float CarrotChaser::compute_distance(const std::vector<float> & a, const std::vector<float> & b)
{
  return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]));
}

std::pair<bool, std::vector<float>> CarrotChaser::update(
  const std::vector<float> & current_position)
{
  if (path_.empty() || completed_) {
    return {false, current_goal_};
  }

  if (compute_distance(current_position, current_goal_) < tol_) {
    float distance = 0.0f;
    int new_idx = current_goal_idx_;
    bool reached_end = false;

    for (int i = current_goal_idx_ + 1; i < static_cast<int>(path_.size()); ++i) {
      distance += compute_distance(path_[i - 1], path_[i]);
      new_idx = i;
      if (distance >= forward_step_) {
        break;
      }
      if (i == static_cast<int>(path_.size()) - 1) {
        reached_end = true;
      }
    }

    current_goal_idx_ = new_idx;
    current_goal_ = path_[current_goal_idx_];
    if (reached_end) {
      completed_ = true;
    }
    return {true, current_goal_};
  }
  return {false, current_goal_};
}

}  // namespace algorithm::carrot_chaser
