#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "pcl_bbox_filter.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<PointCloudBboxFilter>(options));

    rclcpp::shutdown();
    return 0;
}