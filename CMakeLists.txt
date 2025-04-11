cmake_minimum_required(VERSION 3.8)
project(coordinate_transformer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include(FetchContent)

# Add yaml-cpp 

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG master # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_MakeAvailable(yaml-cpp)


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_lint_auto REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS
  include/coordinate_transformer
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  yaml-cpp
  ament_index_cpp
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)






# Add library for coordinate_transformer
add_library(coordinate_transformer
  src/coordinate_transformer.cpp
)

ament_target_dependencies(coordinate_transformer
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_include_directories(coordinate_transformer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link yaml-cpp to the library
# target_link_libraries(coordinate_transformer PUBLIC yaml-cpp::yaml-cpp)

# Add GoogleTest
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG main
)
FetchContent_MakeAvailable(googletest)

enable_testing()

# Add test executable
add_executable(
  coordinate_transformer_test
  test/coordinate_transformer_test.cpp
)

ament_target_dependencies(coordinate_transformer_test
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  ament_index_cpp
)

target_link_libraries(
  coordinate_transformer_test
  GTest::gtest_main
  coordinate_transformer
  yaml-cpp
  ${ament_index_cpp_LIBRARIES} 
)

include(GoogleTest)
gtest_discover_tests(coordinate_transformer_test
  PROPERTIES
    ENVIRONMENT "GTEST_OUTPUT=xml:${CMAKE_BINARY_DIR}/test_results/"
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/Testing/Temporary/LastTest.log"
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Linting
if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)

  find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(${PROJECT_NAME}_tutorial_test test/coordinate_transformer_test.cpp)
  install(DIRECTORY
  test/configs/
  DESTINATION share/${PROJECT_NAME}/test/configs
)
  # Skip linter checks for copyright and cpplint
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
