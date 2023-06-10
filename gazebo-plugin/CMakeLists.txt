cmake_minimum_required(VERSION 3.5)
project(MyIgnitionGazeboPlugin)

# Find the Ignition Gazebo libraries
find_package(ignition-gazebo REQUIRED)

# Set the source files for your plugin
set(SOURCES
    hello_world.cc
)

# Create an executable for your plugin
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Add include directories for Ignition Gazebo headers
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${ignition-gazebo3_INCLUDE_DIRS}
)

# Link against the Ignition Gazebo libraries
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${ignition-gazebo3_LIBRARIES}
)

# Set the C++ standard to C++17
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
