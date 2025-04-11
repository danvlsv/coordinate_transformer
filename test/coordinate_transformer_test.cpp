#include "coordinate_transformer.hpp"

#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

class CoordinateTransformerTest : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<CoordinateTransformer::CoordinateTransformer> transformer;
  std::string package_share_dir;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("coordinate_transformer_test");
    transformer = std::make_shared<CoordinateTransformer::CoordinateTransformer>(node);

    package_share_dir = ament_index_cpp::get_package_share_directory("coordinate_transformer");
  }

  void TearDown() override
  {
    std::remove("/tmp/test_config.yaml");
    rclcpp::shutdown();
  }

  std::string createTempFile(const std::string &content)
  {
    std::string filename = "/tmp/test_config.yaml"; // Use a temporary path
    std::ofstream ofs(filename);
    ofs << content;
    ofs.close();
    return filename;
  }

  std::string getConfigPath(const std::string &config_name)
  {
    return (std::filesystem::path(package_share_dir) / "test" / "configs" / config_name).string();
  }
};

TEST_F(CoordinateTransformerTest, LoadConfig_Success)
{
  // Create a valid config file
  std::string valid_config = R"(
transforms:
  - parent_frame: "world"
    child_frame: "robot_base"
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  - parent_frame: "robot_base"
    child_frame: "frame1"
    translation:
      x: 0.5
      y: 0.5
      z: 0.5
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0

bounds:
  - frame: "frame1"
    min_translation:
      x: -10.0
      y: -10.0
      z: -10.0
    max_translation:
      x: 10.0
      y: 10.0
      z: 10.0
        )";

  std::string config_file = createTempFile(valid_config);

  // Load the configuration
  EXPECT_NO_THROW(transformer->loadConfig(config_file));
}

TEST_F(CoordinateTransformerTest, LoadConfig_NoFile)
{

  // Provide incorrect file path in the test directory
  EXPECT_THROW(transformer->loadConfig("noFile"), CoordinateTransformer::Exceptions::NoFileException);
}

TEST_F(CoordinateTransformerTest, LoadConfig_BadTransform)
{
  // Create a config file
  std::string config = R"(
transforms:
  - parent_frame: "world"
    child_frame: "robot_base"
    translation:
      x: 0.0
      y: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0

                )";

  std::string config_file = createTempFile(config);

  EXPECT_THROW(transformer->loadConfig(config_file), CoordinateTransformer::Exceptions::ParsedBadTransformException);
}

TEST_F(CoordinateTransformerTest, LoadConfig_BadBounds)
{
  // Assuming you have a valid config.yaml file in the test directory
  EXPECT_THROW(transformer->loadConfig(getConfigPath("configBadBounds.yaml")), CoordinateTransformer::Exceptions::ParsedBadBoundsException);
}

TEST_F(CoordinateTransformerTest, LoadConfig_NoTransforms)
{
  // Assuming you have a valid config.yaml file in the test directory
  EXPECT_THROW(transformer->loadConfig(getConfigPath("configNoTransforms.yaml")), CoordinateTransformer::Exceptions::NoTransformsFoundException);
}

TEST_F(CoordinateTransformerTest, LoadConfig_InvalidBounds)
{
  // Assuming you have a valid config.yaml file in the test directory
  EXPECT_THROW(transformer->loadConfig(getConfigPath("configInvalidBounds.yaml")), CoordinateTransformer::Exceptions::ParsedInvalidBoundsException);
}

//
// Convert Tests
//

TEST_F(CoordinateTransformerTest, Convert_InvalidInput)
{

  transformer->loadConfig(getConfigPath("config.yaml"));
  geometry_msgs::msg::PoseStamped input, output;
  input.header.frame_id = ""; // invalid frame
  auto result = transformer->convert(input, output, "robot_base");
  ASSERT_EQ(result, CoordinateTransformer::ResultStatus::INVALID_INPUT);
}

TEST_F(CoordinateTransformerTest, Convert_OutOfBounds)
{
  // Load a valid configuration
  transformer->loadConfig(getConfigPath("configOutOfBounds.yaml"));

  // Create an input PoseStamped message
  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = "robot_base";
  input.pose.position.x = 1.0;
  input.pose.position.y = 2.0;
  input.pose.position.z = 0.0;
  input.pose.orientation.w = 1.0; // No rotation

  // Prepare an output PoseStamped message
  geometry_msgs::msg::PoseStamped output;

  // Perform the coordinate transformation
  auto status = transformer->convert(input, output, "frame1");

  // Check that the transformation was successful
  EXPECT_EQ(status, CoordinateTransformer::ResultStatus::OUT_OF_BOUNDS);
}

TEST_F(CoordinateTransformerTest, Convert_TransformNotFound)
{
  // Load a valid configuration
  transformer->loadConfig(getConfigPath("config.yaml"));

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
  EXPECT_EQ(status, CoordinateTransformer::ResultStatus::TRANSFORM_NOT_FOUND);
}

TEST_F(CoordinateTransformerTest, Convert_Success)
{
  // Load a valid configuration
  transformer->loadConfig(getConfigPath("config.yaml"));

  // Create an input PoseStamped message
  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = "robot_base";
  input.pose.position.x = 1.0;
  input.pose.position.y = 2.0;
  input.pose.position.z = 0.0;
  input.pose.orientation.w = 1.0; // No rotation

  // Prepare an output PoseStamped message
  geometry_msgs::msg::PoseStamped output;

  // Perform the coordinate transformation
  auto status = transformer->convert(input, output, "frame1");

  // Check that the transformation failed
  EXPECT_EQ(status, CoordinateTransformer::ResultStatus::SUCCESS);
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

TEST_F(CoordinateTransformerTest, SetBounds_Success)
{
  transformer->loadConfig(getConfigPath("config.yaml"));
  geometry_msgs::msg::Point min, max;
  min.x = -20;
  min.y = -20;
  min.z = -20;

  max.x = max.y = max.z = std::numeric_limits<double>::infinity();
  EXPECT_NO_THROW(transformer->setBounds("robot_base", min, max));
}

TEST_F(CoordinateTransformerTest, SetBounds_FailureInvertedBounds)
{
  transformer->loadConfig(getConfigPath("config.yaml"));
  geometry_msgs::msg::Point min, max;
  max.x = -20;
  max.y = -20;
  max.z = -20;

  min.x = 40;
  min.y = min.z = -20;
  EXPECT_THROW(transformer->setBounds("robot_base", min, max), CoordinateTransformer::Exceptions::InvertedBoundsException);
}

TEST_F(CoordinateTransformerTest, SetBounds_FailureEqual)
{
  transformer->loadConfig(getConfigPath("config.yaml"));
  geometry_msgs::msg::Point min, max;
  max.x = max.y = max.z = -20;
  min.x = min.y = min.z = -20;
  EXPECT_THROW(transformer->setBounds("robot_base", min, max), CoordinateTransformer::Exceptions::EqualBoundsException);
}

TEST_F(CoordinateTransformerTest, ConvertValue_Correct)
{
  // Load a valid configuration
  transformer->loadConfig(getConfigPath("config.yaml"));

  // Create an input PoseStamped message
  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = "robot_base";
  input.pose.position.x = 1.0;
  input.pose.position.y = 2.0;
  input.pose.position.z = 0.0;
  input.pose.orientation.w = 1.0; // No rotation

  // Prepare an output PoseStamped message
  geometry_msgs::msg::PoseStamped output;

  // Perform the coordinate transformation
  auto status = transformer->convert(input, output, "frame1");

  geometry_msgs::msg::PoseStamped test;
  test.header.frame_id = "frame1";
  test.pose.position.x = 0.5;
  test.pose.position.y = 1.5;
  test.pose.position.z = -0.5;
  test.pose.orientation.w = 1.0; // No rotation

  EXPECT_EQ(test, output);
}

TEST_F(CoordinateTransformerTest, ConvertValue_CorrectBaseAndInverseTransform)
{
  // Load a valid configuration
  transformer->loadConfig(getConfigPath("config.yaml"));

  // Create an input PoseStamped message
  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = "robot_base";
  input.pose.position.x = 1.0;
  input.pose.position.y = 2.0;
  input.pose.position.z = 0.0;
  input.pose.orientation.w = 1.0; // No rotation

  // Prepare an output PoseStamped message
  geometry_msgs::msg::PoseStamped output;

  // Perform the coordinate transformation
  auto status = transformer->convert(input, output, "frame1");

  geometry_msgs::msg::PoseStamped test;
  test.header.frame_id = "frame1";
  test.pose.position.x = 0.5;
  test.pose.position.y = 1.5;
  test.pose.position.z = -0.5;
  test.pose.orientation.w = 1.0; // No rotation

  EXPECT_EQ(test, output);

  output.pose.position.x = 4.25;

  geometry_msgs::msg::PoseStamped newOutput;
  // Perform the coordinate transformation
  auto status2 = transformer->convert(output, newOutput, "robot_base");

  geometry_msgs::msg::PoseStamped test2;
  test2.header.frame_id = "robot_base";
  test2.pose.position.x = 4.75;
  test2.pose.position.y = 2.0;
  test2.pose.position.z = 0.0;
  test2.pose.orientation.w = 1.0; // No rotation

  EXPECT_EQ(test2, newOutput);
}

TEST_F(CoordinateTransformerTest, ConvertValue_CorrectRotation)
{
  transformer->loadConfig(getConfigPath("configRotation.yaml"));

  // Input pose in robot_base frame
  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = "robot_base";
  input.pose.position.x = 1.0;
  input.pose.position.y = 2.0;
  input.pose.position.z = 3.0;
  input.pose.orientation.w = 1.0; // Identity orientation

  // Perform transformation
  geometry_msgs::msg::PoseStamped output;
  auto status = transformer->convert(input, output, "rotated_frame");

  // Verify position transformation
  // 180° X rotation: (x,y,z) → (x,-y,-z)
  EXPECT_NEAR(output.pose.position.x, 1.0, 1e-5);  // X unchanged
  EXPECT_NEAR(output.pose.position.y, -2.0, 1e-5); // Y inverted
  EXPECT_NEAR(output.pose.position.z, -3.0, 1e-5); // Z inverted

  // Verify orientation transformation
  // Identity orientation transformed by 180° X rotation
  EXPECT_NEAR(output.pose.orientation.x, 1.0, 1e-5); // Inverse of 180° X
  EXPECT_NEAR(output.pose.orientation.w, 0.0, 1e-5);
}
