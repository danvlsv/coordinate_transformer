#ifndef COORDINATE_TRANSFORMER_HPP
#define COORDINATE_TRANSFORMER_HPP

#include <rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>

#include <unordered_map.h>
#include <string>

#include <yaml.hpp>

enum class ResultStatus
{
    SUCCESS,
    OUT_OF_BOUNDS,
    TRANSFORM_NOT_FOUND,
    INVALID_INPUT
};

class CoordinateTransformer
{

public:

    CoordinateTransformer(rclcpp::Node::SharedPtr node);
    
    // Прямое преобразование (из исходной системы в целевую)
    ResultStatus convert(
        const geometry_msgs::msg::PoseStamped &input,
        geometry_msgs::msg::PoseStamped &output,
        const std::string &target_frame);

    // Обратное преобразование (из целевой системы в исходную)
    ResultStatus inverseConvert(
        const geometry_msgs::msg::PoseStamped &input,
        geometry_msgs::msg::PoseStamped &output,
        const std::string &source_frame);

    void addTransform(const geometry_msgs::msg::TransformStamped &transform);

    void setBounds(const std::string &frame_id,
                   const geometry_msgs::msg::Point &min,
                   const geometry_msgs::msg::Point &max);

    void loadConfig(const std::string &config_file);

private:

    rclcpp::Node::SharedPtr node;
    tf2_ros::Buffer tf_buffer;
    std::unordered_map<std::string, std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> bounds;
    std::unordered_map<std::string, geometry_msgs::msg::TransformStamped> custom_transforms;
};

#endif