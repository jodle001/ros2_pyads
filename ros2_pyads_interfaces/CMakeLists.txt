cmake_minimum_required(VERSION 3.5)
project(ros2_pyads_interfaces)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
# Add message files if you have any
  "msg/AdsComConfig.msg"
)

set(srv_files
        "srv/GetAdsComConfig.srv"
        "srv/ReadBool.srv"
        "srv/WriteBool.srv"
        "srv/ReadString.srv"
        "srv/ReadByteArray.srv"
        "srv/WriteInt.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        ${srv_files}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
