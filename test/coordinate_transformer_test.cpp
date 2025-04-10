#include "coordinate_transformer.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

class CoordinateTransformerTest : public ::testing::Test
{
protected:
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<CoordinateTransformer> transformer;

    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        node = rclcpp::Node::make_shared("coordinate_transformer_test");
        transformer = std::make_shared<CoordinateTransformer>(node);
    }

    void TearDown() override
    {
        rclcpp::shutdown();
    }
};

TEST_F(CoordinateTransformerTest, LoadConfig_Success)
{
    // Assuming you have a valid config.yaml file in the test directory
    EXPECT_NO_THROW(transformer->loadConfig("/home/danvlsv/ws_moveit/src/coordinate_transformer/config/config.yaml"));
}

TEST_F(CoordinateTransformerTest, LoadConfig_NoFile)
{
    // Assuming you have a valid config.yaml file in the test directory
    EXPECT_THROW(transformer->loadConfig("/home/danvlsv/ws_moveit/src/coordinate_transformer/config/configImaginary.yaml"),YAML::BadFile);
}

TEST_F(CoordinateTransformerTest,LoadConfig_BadTransform)
{
    // Assuming you have a valid config.yaml file in the test directory
    EXPECT_THROW(transformer->loadConfig("/home/danvlsv/ws_moveit/src/coordinate_transformer/config/configInvalid.yaml"),YAML::Exception);
}


TEST_F(CoordinateTransformerTest, Convert_Success)
{
    // Load a valid configuration
    transformer->loadConfig("/home/danvlsv/ws_moveit/src/coordinate_transformer/config/config.yaml");

    // Create an input PoseStamped message
    geometry_msgs::msg::PoseStamped input;
    input.header.frame_id = "robot_base";
    // input.header.stamp = node->get_clock()->now();
    input.pose.position.x = 1.0;
    input.pose.position.y = 2.0;
    input.pose.position.z = 0.0;
    input.pose.orientation.w = 1.0; // No rotation

    // Prepare an output PoseStamped message
    geometry_msgs::msg::PoseStamped output;

    // Perform the coordinate transformation
    auto status = transformer->convert(input, output, "frame1");

    // Check that the transformation was successful
    EXPECT_EQ(status, ResultStatus::SUCCESS);
}

TEST_F(CoordinateTransformerTest, Convert_Failure)
{
    // Load a valid configuration
    transformer->loadConfig("/home/danvlsv/ws_moveit/src/coordinate_transformer/config/config.yaml");

    // Create an input PoseStamped message with an invalid frame
    geometry_msgs::msg::PoseStamped input;
    input.header.frame_id = "invalid_frame"; // This frame does not exist
    input.header.stamp = node->get_clock()->now();
    input.pose.position.x = 1.0;
    input.pose.position.y = 2.0;
    input.pose.position.z = 0.0;
    input.pose.orientation.w = 1.0; // No rotation

    // Prepare an output PoseStamped message
    geometry_msgs::msg::PoseStamped output;

    // Perform the coordinate transformation
    auto status = transformer->convert(input, output, "robot_base");

    // Check that the transformation failed
    EXPECT_EQ(status, ResultStatus::TRANSFORM_NOT_FOUND);
}

TEST_F(CoordinateTransformerTest, AddTransform_Success)
{
    // Create a custom transform
    geometry_msgs::msg::TransformStamped custom_transform;
    custom_transform.header.frame_id = "base_frame";
    custom_transform.child_frame_id = "custom_frame";
    custom_transform.transform.translation.x = 1.0;
    custom_transform.transform.translation.y = 0.0;
    custom_transform.transform.translation.z = 0.0;
    custom_transform.transform.rotation.w = 1.0; // No rotation

    // Add the custom transform
    transformer->addTransform(custom_transform);

}

