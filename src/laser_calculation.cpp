#include "micvision/header.h"


void MicvisionLocalization::scanCallback(const sensor_msgs::msg::LaserScan& scan) {
  RCLCPP_INFO(get_logger(),"scanCallback");  

  mtx.lock();
  if (rclcpp::ok()) {
    point_cloud_.clear();
    float angle = scan.angle_min;
    for (int i = 0; i < scan.ranges.size(); i += laserscan_circle_step_) {
      const float range = scan.ranges[i];
      if (min_valid_range_ <= range && range <= max_valid_range_) {
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        point_cloud_.push_back(rotation * (range * Eigen::Vector3f::UnitX()));
      }

      angle += scan.angle_increment * laserscan_circle_step_;
    }
  }
  mtx.unlock();
}

void MicvisionLocalization::handleLaserScan() {
  mtx.lock();
  laserscan_samples_.clear();
  double angle = -M_PI;
  laserscan_samples_.reserve(static_cast<int>(
                               PI_2 / RADIAN_PRE_DEGREE / laserscan_anglar_step_));
  while (angle <= M_PI) {
    laserscan_samples_.emplace_back(transformPointCloud(
                                      Eigen::Quaternionf(
                                        Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()))));
    angle += laserscan_anglar_step_ * RADIAN_PRE_DEGREE;
  }
  mtx.unlock();
}


bool MicvisionLocalization::validPosition(const int uv, const int index) {     //const const 
  micvision::Pixel start(uv % width_, uv / width_),
        end((uv + index) % width_, (uv + index) / width_);

  const auto line = bresenham(start, end);

  int i;
  for (i = 0; i < line.size(); ++i) {
    const auto l = line[i](0) + line[i](1) * width_;
    if (inflated_map_data_[l].second > 0.5)
      break;
  }

  if (line.size() - i > 5)
    return false;
  else
    return true;
}

double MicvisionLocalization::scoreASample(const LaserScanSample& sample,
                                           const int u, const int v) {
  double score = 0.0;
  if (quick_score_) {
    int step = sample.point_cloud.size() / quick_score_num_;
    int object = 0;
    for (int i = 0; i < sample.point_cloud.size(); i += step) {
      if (current_map_.getRawData(sample.point_cloud[i][0] + v,
                                  sample.point_cloud[i][1] + u) >= 50)
        object++;
    }
    if (object <= quick_score_num_ / 2) return 0;
  }
  for (auto s : sample.point_cloud) {
    score += current_map_.getRawData(s[0] + v, s[1] + u);
  }
  return score;
}