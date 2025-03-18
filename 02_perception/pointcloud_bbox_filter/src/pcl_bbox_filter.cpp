#include "pcl_bbox_filter.hpp"

PointCloudBboxFilter::PointCloudBboxFilter(const rclcpp::NodeOptions options) 
    : Node("bbox_filter_node")
{
    // Declare parameters
    this->declare_parameter("bbox_x_min", 0.0f);
    this->declare_parameter("bbox_x_max", 0.0f);
    this->declare_parameter("bbox_y_min", 0.0f);
    this->declare_parameter("bbox_y_max", 0.0f);
    this->declare_parameter("bbox_z_min", 0.0f);
    this->declare_parameter("bbox_z_max", 0.0f);
    this->declare_parameter("map_path", "");
    this->declare_parameter("pose_topic", "");
    this->declare_parameter("output_cloud_topic", "");
    
    // Get the pose topic
    this->get_parameter("pose_topic", pose_topic_);
    this->get_parameter("output_cloud_topic", output_cloud_topic_);

    // Get the point cloud map path
    this->get_parameter("map_path", map_path_);

    RCLCPP_INFO(get_logger(), "MAP RECEIVING...");
    RCLCPP_INFO(get_logger(), "Map path: %s", map_path_.c_str());

    input_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::io::loadPCDFile<pcl::PointXYZI>(map_path_, *input_cloud_);
    RCLCPP_INFO(get_logger(), "Map Size %ld", input_cloud_->size());
    RCLCPP_INFO(get_logger(), "MAP RECEIVING...[DONE]");
    
    // ROS2 pubsub
    cropped_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_cloud_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable());  //TODO: Check the QoS settings
    
    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&PointCloudBboxFilter::poseCallback, this, std::placeholders::_1));
}

PointCloudBboxFilter::~PointCloudBboxFilter()
{
}

void PointCloudBboxFilter::cropPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &output_cloud, 
    const Eigen::Vector4f &min_point, const Eigen::Vector4f &max_point, 
    const Eigen::Vector3f &translation, const Eigen::Vector3f &euler)
{
    // Initialize the cropbox filter
    pcl::CropBox<pcl::PointXYZI> crop_box_filter(false);
    crop_box_filter.setInputCloud(input_cloud_);
    
    // Set the min, max and pose of box
    crop_box_filter.setMin (min_point);
    crop_box_filter.setMax (max_point);
    crop_box_filter.setTranslation(translation);
    crop_box_filter.setRotation(euler);

    // Crop the point cloud
    crop_box_filter.filter(*output_cloud);
}

void PointCloudBboxFilter::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg)
{
    // Get parameters
    this->get_parameter("bbox_x_min", bbox_x_min_);
    this->get_parameter("bbox_x_max", bbox_x_max_);
    this->get_parameter("bbox_y_min", bbox_y_min_);
    this->get_parameter("bbox_y_max", bbox_y_max_);
    this->get_parameter("bbox_z_min", bbox_z_min_);
    this->get_parameter("bbox_z_max", bbox_z_max_);
    
    // Get the pose and define the min and max points for the bounding box
    float curr_global_pos_x = static_cast<float>(pose_msg->pose.pose.position.x);
    float curr_global_pos_y = static_cast<float>(pose_msg->pose.pose.position.y);
    float curr_global_pos_z = static_cast<float>(pose_msg->pose.pose.position.z);

    float curr_global_quat_x = static_cast<float>(pose_msg->pose.pose.orientation.x);
    float curr_global_quat_y = static_cast<float>(pose_msg->pose.pose.orientation.y);
    float curr_global_quat_z = static_cast<float>(pose_msg->pose.pose.orientation.z);
    float curr_global_quat_w = static_cast<float>(pose_msg->pose.pose.orientation.w);


    Eigen::Vector4f min_point{bbox_x_min_, bbox_y_min_, bbox_z_min_, 1.0f};
    Eigen::Vector4f max_point{bbox_x_max_, bbox_y_max_, bbox_z_max_, 1.0f};
    
    Eigen::Vector3f translation{curr_global_pos_x, curr_global_pos_y, curr_global_pos_z};
    
    // extract the inverse of the pose (map to curr_pose) i.e. conjugate
    Eigen::Quaternion<float> quaternion{curr_global_quat_w, -curr_global_quat_x, -curr_global_quat_y, 
                                        -curr_global_quat_z};
    Eigen::Vector3f euler = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

    // Crop the point cloud and publish it
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    cropPointCloud(output_cloud, min_point, max_point, translation, euler);
    
    sensor_msgs::msg::PointCloud2 output_cloud_msg;
    pcl::toROSMsg(*output_cloud, output_cloud_msg);
    
    // set the frame id and other parameters
    output_cloud_msg.header.frame_id = "map";
    output_cloud_msg.header.stamp = this->get_clock()->now();

    cropped_cloud_publisher_->publish(output_cloud_msg);
}