cmake_minimum_required(VERSION 3.5)
project(lcd_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lcd_msgs REQUIRED)


################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  "rclcpp"
  "lcd_msgs"
)

set(EXEC_NAME "lcd_driver")


add_executable(${EXEC_NAME}
  src/lcd_driver.cpp
  src/lcd_driver_node.cpp
)

option(SIMULATION "Simulation mode" OFF)


if(SIMULATION)
  add_definitions(-DSIMULATION)
else()
  target_sources(${EXEC_NAME} PRIVATE
    src/I2CIO.cpp
    src/LCD.cpp
    src/LiquidCrystal_I2C.cpp
  )
  target_link_libraries(${EXEC_NAME} i2c)
endif(SIMULATION)


ament_target_dependencies(${EXEC_NAME} ${dependencies})


################################################################################
# Install
################################################################################
install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(lcd_msgs)
ament_package()
