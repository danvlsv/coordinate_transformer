#include "coordinate_transformer.hpp"

ResultStatus CoordinateTransformer::convert(const geometry_msgs::msg::PoseStamped &input, geometry_msgs::msg::PoseStamped &output, const std::string &target_frame)
{
    try
    {
        // Используем tf2 для преобразования
        output = tf_buffer.transform(input, target_frame);
        return ResultStatus::SUCCESS;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CoordinateTransformer"), "Transform failed: %s", ex.what());
        return ResultStatus::TRANSFORM_NOT_FOUND;
    }
}

ResultStatus CoordinateTransformer::inverseConvert(const geometry_msgs::msg::PoseStamped &input, geometry_msgs::msg::PoseStamped &output, const std::string &source_frame)
{
    try
    {
        // Используем tf2 для преобразования
        output = tf_buffer.transform(input, source_frame);
        return ResultStatus::SUCCESS;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CoordinateTransformer"), "Transform failed: %s", ex.what());
        return ResultStatus::TRANSFORM_NOT_FOUND;
    }
}

void CoordinateTransformer::addTransform(const geometry_msgs::msg::TransformStamped &transform)
{
    custom_transforms[transform.child_frame_id] = transform;
}

void CoordinateTransformer::setBounds(const std::string &frame_id, const geometry_msgs::msg::Point &min, const geometry_msgs::msg::Point &max)
{
    bounds[frame_id] = std::make_pair(min, max);
}

void CoordinateTransformer::loadConfig(const std::string &config_file)
{
    YAML::Node config = YAML::LoadFile(config_file);
    for (const auto &transform : config["transforms"])
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = transform["parent_frame"].as<std::string>();
        tf.child_frame_id = transform["child_frame"].as<std::string>();
        tf.transform.translation.x = transform["translation"]["x"].as<double>();
        tf.transform.translation.y = transform["translation"]["y"].as<double>();
        tf.transform.translation.z = transform["translation"]["z"].as<double>();
        tf.transform.rotation.x = transform["rotation"]["x"].as<double>();
        tf.transform.rotation.y = transform["rotation"]["y"].as<double>();
        tf.transform.rotation.z = transform["rotation"]["z"].as<double>();
        tf.transform.rotation.w = transform["rotation"]["w"].as<double>();

        addTransform(tf);
    }
}