#include "micvision/header.h"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MicvisionLocalization>();
  node->getMap();
  node->inflateMap();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}