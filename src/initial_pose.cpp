#include "micvision/header.h"


std::vector<micvision::Pixel> MicvisionLocalization::bresenham(const micvision::Pixel& start,
                                                    const micvision::Pixel& end) {
                                                  
  int x = start(0), y = start(1);
  int dx = abs(end(0) - x);
  int dy = abs(end(1) - y);

  const int ux = end(0) > x ? 1 : -1;
  const int uy = end(1) > y ? 1 : -1;

  bool exchange = false;      // exchange dx and dy?
  if (dy > dx) {
    std::swap(dx, dy);
    exchange = true;
  }

  int p = 2 * dy - dx;
  std::vector<micvision::Pixel> result;

  for (int i = 0; i <= dx; ++i) {
    if (x <= 5 || x >= current_map_.getWidth() - 5 ||
        y <= 5 || y >= current_map_.getHeight() - 5)
      break;
    result.push_back(micvision::Pixel(x, y));
    if (p >= 0) {
      if (!exchange)
        y += uy;
      else
        x += ux;
      p -= 2 * dx;
    }

    if (!exchange)
      x += ux;
    else
      y += uy;
    p += 2 * dy;
  }
  return result;
}

inline micvision::Pixel floor(const Eigen::Vector3f& v) {
  return micvision::Pixel(std::lround(v[0] - 0.5), std::lround(v[1] - 0.5));
}

struct LaserScanSample MicvisionLocalization::transformPointCloud(
  const Eigen::Quaternionf& transform) {
  micvision::PointCloudUV result;
  result.reserve(point_cloud_.size());
  std::vector<int> indices;
  indices.reserve(point_cloud_.size());
  const float resolution = current_map_.getResolution();
  int min_x = width_, max_x = -min_x;
  int min_y = height_, max_y = -min_y;
  for (const Eigen::Vector3f& point : point_cloud_) {
    // result.emplace_back(transform * point);
    const micvision::Pixel temp = floor(transform * point / resolution);
    min_x = min_x < temp[0] ? min_x : temp[0];
    min_y = min_y < temp[1] ? min_y : temp[1];
    max_x = max_x > temp[0] ? max_x : temp[0];
    max_y = max_y > temp[1] ? max_y : temp[1];
    result.emplace_back(temp);
    indices.emplace_back(temp[0] + temp[1] * width_);
  }
  return LaserScanSample{result, indices, min_x, max_x, min_y, max_y};
}

void MicvisionLocalization::delete_params() {
  delete[] inflation_markers_;
  delete[] cached_distances_;
  delete[] cached_costs_;
}


void MicvisionLocalization::init_pose_callback() {
  handleLaserScan();
  RCLCPP_INFO(get_logger(),"start score");
  double score = 0.5;
  int count = 0;

  const int sample_size = laserscan_samples_[0].point_cloud.size();
  const int step = sample_size / quick_score_num_;
  for (int v = 0; v < height_; v += range_step_) {
    for (int u = 0; u < width_; u += range_step_) {
      const int uv = v * width_ + u;
      if (!inflated_map_data_[uv].first) {
        continue;
      }

      for (int i = 0; i < laserscan_samples_.size(); ++i) {
        const LaserScanSample& sample = laserscan_samples_[i];
        if (u + sample.min_x <= 1 || u + sample.max_x >= width_ - 1)
          continue;
        if (v + sample.min_y <= 1 || v + sample.max_y >= height_ - 1)
          continue;

        double sample_score = 0;
        if (quick_score_) {
          int object = 0;
          for (int point_index = 0; point_index < sample_size;
               point_index += step) {
            if (inflated_map_data_[sample.indices[point_index]
                                   + uv].second >= 0.5) {
              ++object;
            }
          }

          if (object <= quick_score_num_ / 2)
            continue;
        }
        count++;

        for (auto point_index : sample.indices)
          sample_score += inflated_map_data_[point_index + uv].second;
        sample_score /= static_cast<double>(sample_size);

        if (sample_score > score) {
          if (quick_score_) {
            int object = 0;
            for (int point_index = 0; point_index < sample_size;
                 point_index += step) {
              if (validPosition(uv, sample.indices[point_index]))
                ++object;
            }
            if (object <= quick_score_num_ / 2)
              continue;
          }

          score = sample_score;
          best_angle_ =  i * laserscan_anglar_step_ * RADIAN_PRE_DEGREE;
          best_position_ = micvision::Pixel(u, v);
        }
      }
    }
  }
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
  double qx, qy, qz, qw;
  RCLCPP_INFO(get_logger(), "best_position_[x] = %d, best_position_[y] = %d", best_position_[0], best_position_[1]);
  const double x = best_position_[0]*resolution_ + current_map_.getOriginX();
  const double y = best_position_[1]*resolution_ + current_map_.getOriginY();

  tf2::Quaternion q;
  q.setRPY(0, 0, best_angle_);            
  qx = q.x();
  qy = q.y();
  qz = q.z();
  qw = q.w();

  rclcpp::Time now = this->get_clock()->now();

  init_pose.header.stamp = now;
  init_pose.header.frame_id = "map";

  init_pose.pose.pose.position.x = x;
  init_pose.pose.pose.position.y = y;
  init_pose.pose.pose.position.z = 0;

  init_pose.pose.pose.orientation.x = qx;
  init_pose.pose.pose.orientation.y = qy;
  init_pose.pose.pose.orientation.z = qz;
  init_pose.pose.pose.orientation.w = qw;

  init_pose.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                               0.0,  0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0,  0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0,  0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0,  0.0, 0.0, 0.0, 0.0, 0.06853891945200942
                              };
  publisher_->publish(init_pose);

  RCLCPP_INFO(get_logger(),"Best score: %f, angle: %f, Best position: %f, %f, count: %d",
           score, best_angle_ / RADIAN_PRE_DEGREE, x, y, count);

    if(best_position_[0]!=0){
    rclcpp::shutdown();
  }
}
