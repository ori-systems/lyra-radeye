#include "radeye/radeye.h"
#include "rclcpp/rclcpp.hpp"

//Basic node to run radeye sensor

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // The class was renamed to RadEyeSensorNode to avoid conflicts
    auto node = std::make_shared<RadEyeSensorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
