cmake_minimum_required(VERSION 3.10)
project(ParticleFilter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen3
find_package(Eigen3 REQUIRED NO_MODULE)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

# Gather source files
file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)

# Add executable
add_executable(particle_filter_motion_upgrade ${SOURCES})

# Link Eigen
target_link_libraries(particle_filter_motion_upgrade PRIVATE Eigen3::Eigen)
