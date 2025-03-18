// standard C++ header files
#include <memory>
#include <string>

// ROS2 C++ header files
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include"geometry_msgs/msg/pose_with_covariance_stamped.hpp"
// #include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>

// PCL C++ header files
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

class PointCloudBboxFilter : public rclcpp::Node 
{
// For variables
private:
    // Primitive variables
    float bbox_x_min_ = 0.0f;
    float bbox_x_max_ = 0.0f;
    float bbox_y_min_ = 0.0f;
    float bbox_y_max_ = 0.0f;
    float bbox_z_min_ = 0.0f;
    float bbox_z_max_ = 0.0f;
    std::string map_path_ = "";
    std::string pose_topic_ = "";
    std::string output_cloud_topic_ = "";

    // ROS2 variables
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_cloud_publisher_ = nullptr;

    // PCL variables
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ = nullptr;

// For functions
private:
    // void cropPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &output_cloud, 
    //     const Eigen::Vector4f &min_point, const Eigen::Vector4f &max_point);

    // void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg);

    void cropPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &output_cloud, 
        const Eigen::Vector4f &min_point, const Eigen::Vector4f &max_point,
        const Eigen::Vector3f &translation, const Eigen::Vector3f &euler);

    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg);

public:
    PointCloudBboxFilter(const rclcpp::NodeOptions options);
    
    ~PointCloudBboxFilter();
};