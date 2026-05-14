cmake_minimum_required(VERSION 3.0.2)
project(usb_cam_rt)

# ---------------------------------------------------------------------------
# Compiler settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -O3 + architecture-specific tuning for Cortex-A76 (Orange Pi 5 Max).
# -march=armv8.2-a covers the A76 ISA including LSE atomics.
# Fallback: remove the march flag for other hosts.
add_compile_options(
    -O3
    -march=armv8.2-a
    -mtune=cortex-a76
    -fno-strict-aliasing
    -Wall
    -Wextra
)

# ---------------------------------------------------------------------------
# catkin dependencies
# ---------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

# ---------------------------------------------------------------------------
# Includes
# ---------------------------------------------------------------------------
include_directories(
    src
    ${catkin_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# Target: usb_cam_node
# ---------------------------------------------------------------------------
add_executable(usb_cam_node
    src/usb_cam_node.cpp
    src/v4l2_capture.cpp
)

target_link_libraries(usb_cam_node
    ${catkin_LIBRARIES}
    pthread   # -lpthread for POSIX threads + mlockall
    rt        # -lrt for clock_gettime on older libc
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS usb_cam_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
