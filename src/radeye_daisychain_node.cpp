#include "radeye/radeye_daisychain.h"
#include "rclcpp/rclcpp.hpp"

//Basic node to run radeye sensor in daisychain mode

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // The class was renamed to RadEyeDaisychainNode to avoid conflicts
    auto node = std::make_shared<RadEyeDaisychainNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}