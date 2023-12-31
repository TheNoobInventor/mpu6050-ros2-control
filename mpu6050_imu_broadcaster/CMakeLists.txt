cmake_minimum_required(VERSION 3.8)
project(mpu6050_imu_broadcaster)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# COMPILE
add_library(mpu6050_imu_broadcaster SHARED
  src/mpu6050_imu_broadcaster.cpp
  src/mpu6050_lib.cpp
)

target_include_directories(mpu6050_imu_broadcaster PRIVATE	include)

ament_target_dependencies(mpu6050_imu_broadcaster ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Link i2c library to mpu6050_imu_broadcaster target
target_link_libraries(mpu6050_imu_broadcaster i2c)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface mpu6050_imu_broadcaster.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/mpu6050_imu_broadcaster
)

install(
  TARGETS mpu6050_imu_broadcaster
  DESTINATION share/${PROJECT_NAME}
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# EXPORTS
ament_export_libraries(mpu6050_imu_broadcaster)

include_directories(include)

# Create Cpp executable
add_executable(mpu6050_offsets src/mpu6050_lib.cpp src/mpu6050_offsets.cpp)
add_executable(mpu6050_covariances src/mpu6050_lib.cpp src/mpu6050_covariances.cpp)

# Install Cpp executables
install(TARGETS
  mpu6050_offsets
  mpu6050_covariances
  DESTINATION lib/${PROJECT_NAME}
)

# Link i2c to mpu6050_offsets and mpu6050_covariances targets
target_link_libraries(mpu6050_offsets i2c)
target_link_libraries(mpu6050_covariances i2c)

ament_package()
