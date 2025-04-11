# Coordinate Transformer

A ROS 2 package for managing coordinate transforms and bounds checking between reference frames.

## Prerequisites

- **ROS 2 Jazzy** 
- **Workspace** configured with `colcon`
- **C++17** compatible compiler
- **Dependencies**:
  - `tf2_ros`
  - `geometry_msgs`
  - `yaml-cpp`


## Building

1) source your ROS2 workspace and environment (source install/setup.bash)
2) colcon build --packages-select coordinate_transformer

## Testing

1) source your ROS2 workspace and environment (source install/setup.bash)
2) colcon test --packages-select coordinate_transformer --event-handlers console_direct+

## Usage Example

```cpp
#include "coordinate_transformer.hpp"
#include <rclcpp/rclcpp.hpp>

int main() {

    rclcpp::Node::SharedPtr node;
    std::shared_ptr<CoordinateTransformer::CoordinateTransformer> transformer;

    rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("coordinate_transformer_test");
    transformer = std::make_shared<CoordinateTransformer::CoordinateTransformer>(node);

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

    if (status)
    {
        output.pose.position.x = 4.25;

        geometry_msgs::msg::PoseStamped newOutput;
        // Perform the coordinate transformation
        auto status2 = transformer->convert(output, newOutput, "robot_base");
    }
    

}

```

## Config Example

Define transforms with proper frame names and normalized rotatation quaternion. 

Add box bounds to defined frames, if your project need them

```yaml
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
```


