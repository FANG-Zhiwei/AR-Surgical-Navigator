cmake_minimum_required(VERSION 3.5)
project(mri_tomo_display)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ITK REQUIRED)
find_package(unity_robotics_demo_msgs REQUIRED)

# Include directories
include_directories(
  # ${OpenCV_INCLUDE_DIRS}
  /usr/local/include/opencv4/opencv2/
  ${ITK_INCLUDE_DIRS}
)
include( ${ITK_USE_FILE} )

# Add executable
add_executable(mri_image mri_image_process.cpp)
# Link libraries
ament_target_dependencies(mri_image rclcpp sensor_msgs OpenCV unity_robotics_demo_msgs)
target_link_libraries(mri_image ${OpenCV_LIBS} ${ITK_LIBRARIES})

# Add executable
add_executable(oct_image oct_image.cpp)
# Link libraries
ament_target_dependencies(oct_image rclcpp unity_robotics_demo_msgs OpenCV)
target_link_libraries(oct_image ${OpenCV_LIBS})

install(TARGETS
  mri_image
  oct_image
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
