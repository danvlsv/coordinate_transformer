#include "coordinate_transformer.hpp"


CoordinateTransformer::CoordinateTransformer(rclcpp::Node::SharedPtr node)
    : node(node),tf_buffer(node->get_clock()),tf_listener(tf_buffer){}


ResultStatus CoordinateTransformer::convert(const geometry_msgs::msg::PoseStamped &input, geometry_msgs::msg::PoseStamped &output, const std::string &target_frame)
{
    try
    {
        // Используем tf2 для преобразования
        output = tf_buffer.transform(input, target_frame);

        if (bounds.count(target_frame)) {
            const auto& [min, max] = bounds[target_frame];
            if (output.pose.position.x < min.x || output.pose.position.x > max.x ||
                output.pose.position.y < min.y || output.pose.position.y > max.y ||
                output.pose.position.z < min.z || output.pose.position.z > max.z) {
                RCLCPP_WARN(rclcpp::get_logger("CoordinateTransformer"), "Transformed point out of bounds in frame '%s'", target_frame.c_str());
                return ResultStatus::OUT_OF_BOUNDS;
            }
        }
        

        return ResultStatus::SUCCESS;
    }
    catch (const tf2::InvalidArgumentException& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("CoordinateTransformer"),"Invalid argument: %s", ex.what());
        return ResultStatus::INVALID_INPUT;
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

        if (bounds.count(source_frame)) {
            const auto& [min, max] = bounds[source_frame];
            if (output.pose.position.x < min.x || output.pose.position.x > max.x ||
                output.pose.position.y < min.y || output.pose.position.y > max.y ||
                output.pose.position.z < min.z || output.pose.position.z > max.z) {
                RCLCPP_WARN(rclcpp::get_logger("CoordinateTransformer"), "Transformed point out of bounds in frame '%s'", source_frame.c_str());
                return ResultStatus::OUT_OF_BOUNDS;
            }
        }
        

        return ResultStatus::SUCCESS;
        
    }
    catch (const tf2::InvalidArgumentException& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("CoordinateTransformer"),"Invalid argument: %s", ex.what());
        return ResultStatus::INVALID_INPUT;
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

    // Add to tf buffer
    try {
        tf_buffer.setTransform(transform, "coordinate_transformer", true);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(
            rclcpp::get_logger("CoordinateTransformer"),
            "Failed to set transform: %s", ex.what());
    }
}


void CoordinateTransformer::setBounds(const std::string &frame_id, const geometry_msgs::msg::Point &min, const geometry_msgs::msg::Point &max)
{

    if (min.x > max.x) {
        throw InvertedBoundsException("X");
    }
    if (min.y > max.y) {
        throw InvertedBoundsException("Y");
    }
    if (min.z > max.z) {
        throw InvertedBoundsException("Z");
    }


    if (min!=max)
    {
        if (!bounds.count(frame_id))
        {
            bounds[frame_id] = std::make_pair(min, max);
        }
        else
        {
            bounds[frame_id].first = min;
            bounds[frame_id].second = max;
        }
        
    }
    else {
        throw EqualBoundsException();
    }
    
}

void CoordinateTransformer::loadConfig(const std::string &config_file)
{
    YAML::Node config ;
    try {
       config = YAML::LoadFile(config_file);
    }
    catch (const YAML::BadFile &ex)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("CoordinateTransformer"),
            "Failed to load file %s", ex.what());
        throw NoFileException(config_file);
        return;
    }
    
    
    if (config["transforms"]) {
        for (const auto& transform : config["transforms"]) {
            try {
                geometry_msgs::msg::TransformStamped tf;
                tf.header.frame_id = transform["parent_frame"].as<std::string>();
                tf.child_frame_id = transform["child_frame"].as<std::string>();
                
                auto translation = transform["translation"];
                tf.transform.translation.x = translation["x"].as<double>();
                tf.transform.translation.y = translation["y"].as<double>();
                tf.transform.translation.z = translation["z"].as<double>();
                
                auto rotation = transform["rotation"];
                tf.transform.rotation.x = rotation["x"].as<double>();
                tf.transform.rotation.y = rotation["y"].as<double>();
                tf.transform.rotation.z = rotation["z"].as<double>();
                tf.transform.rotation.w = rotation["w"].as<double>();
                
                addTransform(tf);
            } catch (const YAML::Exception& ex) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("CoordinateTransformer"),
                    "Failed to parse transform: %s", ex.what());
                throw ParsedBadTransformException();
                continue;  
            }
        }
    }
    else{
        throw NoTransformsFoundException();
        return;
    }

    if (config["bounds"]) {
        for (const auto& bound : config["bounds"]) {
            try {
                std::string frame_id = bound["frame"].as<std::string>();
                
                auto min_trans = bound["min_translation"];
                geometry_msgs::msg::Point min;
                min.x = min_trans["x"].as<double>();
                min.y = min_trans["y"].as<double>();
                min.z = min_trans["z"].as<double>();
                
                auto max_trans = bound["max_translation"];
                geometry_msgs::msg::Point max;
                max.x = max_trans["x"].as<double>();
                max.y = max_trans["y"].as<double>();
                max.z = max_trans["z"].as<double>();
                
                setBounds(frame_id, min, max);
            } catch (const YAML::Exception& ex) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("CoordinateTransformer"),
                    "Failed to parse bounds: %s", ex.what());
                throw ParsedBadBoundsException();
                continue;  // Skip this bound but continue with others
            } catch (const BoundValidationException& ex) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("CoordinateTransformer"),
                    "Invalid bounds configuration: %s", ex.what());
                throw ParsedInvalidBoundsException();
                continue;
            }
        }
    }
}