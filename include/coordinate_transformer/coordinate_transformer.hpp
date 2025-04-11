#ifndef COORDINATE_TRANSFORMER_HPP
#define COORDINATE_TRANSFORMER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <unordered_map>
#include <string>

#include <yaml-cpp/yaml.h>
#include <stdexcept>

/**
 * @brief Coordinate transformation utilities for robotic systems
 */
namespace CoordinateTransformer
{

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
        /// @brief Constructs a new Coordinate Transformer instance
        ///
        /// @param[in] node Shared pointer to the ROS 2 node (must not be null)
        CoordinateTransformer(rclcpp::Node::SharedPtr node);

        /// @brief Transforms a pose from its current frame to the target frame
        /// @param input Input pose to transform (must contain valid header.frame_id)
        /// @param output Transformed pose in the target frame
        /// @param target_frame The frame to transform the input pose into
        /// @return ResultStatus indicating:
        ///         - SUCCESS if transform succeeded
        ///         - INVALID_INPUT if frames are empty or pose is invalid
        ///         - TRANSFORM_NOT_FOUND if transform unavailable
        ///         - OUT_OF_BOUNDS if transformed pose exceeds configured bounds
        ResultStatus convert(
            const geometry_msgs::msg::PoseStamped &input,
            geometry_msgs::msg::PoseStamped &output,
            const std::string &target_frame);


        /// @brief Transforms a pose from its current frame to the source frame
        /// @param input Input pose to transform (must contain valid header.frame_id)
        /// @param output Transformed pose in the source frame
        /// @param source_frame The frame to transform the input pose into
        /// @return ResultStatus indicating:
        ///         - SUCCESS if transform succeeded
        ///         - INVALID_INPUT if frames are empty or pose is invalid
        ///         - TRANSFORM_NOT_FOUND if transform unavailable
        ///         - OUT_OF_BOUNDS if transformed pose exceeds configured bounds
        ResultStatus inverseConvert(
            const geometry_msgs::msg::PoseStamped &input,
            geometry_msgs::msg::PoseStamped &output,
            const std::string &source_frame);

        /// @brief Adds a static transform to the internal transform map
        /// @param transform The transform to add, containing:
        ///            - header.frame_id (parent frame)
        ///            - child_frame_id (child frame)
        ///            - transform.translation (x,y,z offsets)
        ///            - transform.rotation (quaternion orientation)
        /// @note Overwrites any existing transform for the same frame pair
        /// @note The transform will be used for all future conversions between these frames
        void addTransform(const geometry_msgs::msg::TransformStamped &transform);

        /// @brief Sets box constraints for a specified frame
        /// @param frame_id The frame these bounds apply to
        /// @param min Minimum corner point of the bounding box (x,y,z coordinates)
        /// @param max Maximum corner point of the bounding box (x,y,z coordinates)
        void setBounds(const std::string &frame_id,
                       const geometry_msgs::msg::Point &min,
                       const geometry_msgs::msg::Point &max);

        /// @brief Loads transform and bounds configuration from a YAML file
        /// @param config_file Path to the YAML configuration file with:
        ///            - "transforms": List of transform definitions
        ///              - parent_frame: Parent frame ID (string)
        ///              - child_frame: Child frame ID (string)
        ///              - translation: {x, y, z} offsets (meters)
        ///              - rotation: {x, y, z, w} quaternion components
        ///            - "bounds": List of bounding box definitions
        ///              - frame: Target frame ID (string)
        ///              - min_translation: {x, y, z} minimum bounds
        ///              - max_translation: {x, y, z} maximum bounds
        /// @note Merges with existing configuration (does not clear previous transforms/bounds)
        /// @note Overwrites existing entries with matching frame IDs
        void loadConfig(const std::string &config_file);

    private:
        rclcpp::Node::SharedPtr node;
        tf2_ros::Buffer tf_buffer;
        std::unordered_map<std::string, std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> bounds;
        std::unordered_map<std::string, geometry_msgs::msg::TransformStamped> custom_transforms;
        tf2_ros::TransformListener tf_listener;
    };

    namespace Exceptions
    {

        // Bounds Exceptions

        class BoundValidationException : public std::runtime_error
        {
        public:
            explicit BoundValidationException(const std::string &what_arg)
                : std::runtime_error(what_arg) {}
        };

        class InvalidBoundException : public BoundValidationException
        {
        public:
            explicit InvalidBoundException(const std::string &what_arg)
                : BoundValidationException("Invalid bound: " + what_arg) {}
        };

        class InvertedBoundsException : public BoundValidationException
        {
        public:
            explicit InvertedBoundsException(const std::string &axis)
                : BoundValidationException("Inverted bounds on axis " + axis) {}
        };

        class EqualBoundsException : public BoundValidationException
        {
        public:
            explicit EqualBoundsException()
                : BoundValidationException("Minimal and max bounds are equal") {}
        };

        // YAML Parsing Exceptions

        class ParsingException : public std::runtime_error
        {
        public:
            explicit ParsingException(const std::string &what_arg)
                : std::runtime_error(what_arg) {}
        };

        class NoFileException : public ParsingException
        {
        public:
            explicit NoFileException(const std::string &filePath)
                : ParsingException("File not found " + filePath) {}
        };

        class NoTransformsFoundException : public ParsingException
        {
        public:
            explicit NoTransformsFoundException()
                : ParsingException("No valid tranforms found") {}
        };

        class ParsedBadTransformException : public ParsingException
        {
        public:
            explicit ParsedBadTransformException()
                : ParsingException("Transform is invalid") {}
        };

        class ParsedBadBoundsException : public ParsingException
        {
        public:
            explicit ParsedBadBoundsException()
                : ParsingException("Error while parsing bounds") {}
        };

        class ParsedInvalidBoundsException : public ParsingException
        {
        public:
            explicit ParsedInvalidBoundsException()
                : ParsingException("Error while adding bounds, check your bounds definition") {}
        };
    }

}

#endif