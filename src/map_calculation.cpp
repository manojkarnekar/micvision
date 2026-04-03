#include "micvision/header.h"


void MicvisionLocalization::odomCallback(const nav_msgs::msg::Odometry& odom) {
  big_angle_twist_ =
    (std::abs(odom.twist.twist.angular.z) >= 0.1) ? true : false;
}


void MicvisionLocalization::getMap() {

    while (!client->wait_for_service(1s)) {

    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) 
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "got the /map_server/map");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /map_server/map");
    }

    current_map_.update(result.get()->map);

    RCLCPP_INFO(get_logger(),"Now inflation the static map.");
    cell_inflation_radius_ = inflation_radius_ / current_map_.getResolution();
    computeCaches();

}

void MicvisionLocalization::computeCaches() {
  cached_costs_ = new signed char* [cell_inflation_radius_ + 2];
  cached_distances_ = new double*[cell_inflation_radius_ + 2];

  for (unsigned int i = 0; i < cell_inflation_radius_ + 2; i++) {
    cached_costs_[i] = new signed char[cell_inflation_radius_ + 2];
    cached_distances_[i] = new double[cell_inflation_radius_ + 2];
    for (unsigned int j = 0; j < cell_inflation_radius_ + 2; j++) {
      double d = sqrt(static_cast<double>(i * i + j * j));
      cached_distances_[i][j] = d;
      d /= static_cast<double>(cell_inflation_radius_);
      if (d > 1) d = 1;
      cached_costs_[i][j] = (1.0 - d) * cost_obstacle_;
    }
  }
}

void MicvisionLocalization::inflateMap() {
  RCLCPP_INFO(get_logger(),"Infalting the map ...");
  const int map_size = current_map_.getSize();
  width_ = current_map_.getWidth();
  height_ = current_map_.getHeight();
  resolution_ = current_map_.getResolution();
  cout<<"width = "<<width_<<" height = "<<height_<<" resolution = "<<resolution_<<endl;

  if (inflation_markers_)
    delete[] inflation_markers_;
  inflation_markers_ = new unsigned char[map_size];
  memset(inflation_markers_, 0, map_size * sizeof(unsigned char));

  while (!inflation_queue_.empty())
    inflation_queue_.pop();

  for (int index = 0; index < map_size; index++) {
    if (current_map_.getData(index) > 0) {
      unsigned int x, y;
      current_map_.getCoordinates(x, y, index);
      enqueueObstacle(index, x, y);
    }
  }

  int count = 0;
  while (!inflation_queue_.empty()) {
    const CellData cell = inflation_queue_.top();
    inflation_queue_.pop();

    unsigned int x, y;
    if (!current_map_.getCoordinates(x, y, cell.index))
      continue;

    if (x >= 1)
      enqueueObstacle(cell.index - 1, cell.x, cell.y);

    if (x < width_ - 1)
      enqueueObstacle(cell.index + 1, cell.x, cell.y);

    if (y >= 1)
      enqueueObstacle(cell.index - width_, cell.x, cell.y);

    if (y < current_map_.getHeight() - 1)
      enqueueObstacle(cell.index + width_, cell.x, cell.y);
    count++;
  }
  RCLCPP_INFO(get_logger(),"Inflated %i cells", count);

  inflated_map_data_.reserve(height_ * width_);
  for (int u = 0; u < height_; ++u) {
    for (int v = 0; v < width_; ++v) {
      const auto data = current_map_.getData(v, u);
      if (data >= 0 && data < 50) {
        inflated_map_data_.emplace_back(
          std::make_pair(true,
                         static_cast<double>(data) /
                         static_cast<double>(cost_obstacle_)));
      } else
        inflated_map_data_.emplace_back(
          std::make_pair(false,
                         static_cast<double>(data) /
                         static_cast<double>(cost_obstacle_)));
    }
  }
}


void MicvisionLocalization::enqueueObstacle(const unsigned int index,
                                            const unsigned int x,
                                            const unsigned int y) {
  unsigned int mx, my;
  if (!current_map_.getCoordinates(mx, my, index) ||
      inflation_markers_[index] != 0)
    return;

  const double d = distanceLookup(mx, my, x, y);
  if (d > cell_inflation_radius_)
    return;

  const CellData cell(d, index, x, y);
  inflation_queue_.push(cell);
  inflation_markers_[index] = 1;
  const signed char value = costLookup(mx, my, x, y);
  current_map_.setData(index, value);
}

inline double MicvisionLocalization::distanceLookup(
  const unsigned int mx, const unsigned int my,
  const unsigned int sx, const unsigned int sy) const {
  const unsigned int dx = abs(static_cast<int>(mx) - static_cast<int>(sx));
  const unsigned int dy = abs(static_cast<int>(my) - static_cast<int>(sy));

  if (dx > cell_inflation_radius_ + 1 || dy > cell_inflation_radius_ + 1) {
    RCLCPP_INFO(get_logger(),"Error in distanceLookup table! Asked for"
              " (%d, %d), but cell_inflation_radius_ is %d!",
              dx, dy, cell_inflation_radius_);
    return cell_inflation_radius_ + 1;
  }
  return cached_distances_[dx][dy];
}

inline signed char MicvisionLocalization::costLookup(
  const unsigned int mx, const unsigned int my,
  const unsigned int sx, const unsigned int sy) const {
  const unsigned int dx = abs(static_cast<int>(mx) - static_cast<int>(sx));
  const unsigned int dy = abs(static_cast<int>(my) - static_cast<int>(sy));

  if (dx > cell_inflation_radius_ + 1 || dy > cell_inflation_radius_ + 1) {
    RCLCPP_INFO(get_logger(),"Error in distanceLookup table! Asked for"
              " (%d, %d), but cell_inflation_radius_ is %d!",
              dx, dy, cell_inflation_radius_);
    return 0;
  }
  return cached_costs_[dx][dy];
}