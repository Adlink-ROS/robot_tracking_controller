cmake_minimum_required(VERSION 3.5)
project(tf_ai_tracker)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(vision_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(object_msgs REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS
	common
)
find_package(visualization_msgs REQUIRED)
# find_package(pcl_conversions REQUIRED)

set(library_name tf_ai_tracker_lib)

include_directories(
  include
  ${PCL_COMMON_INCLUDE_DIRS}
)

add_executable( target_tracker2 src/target_tracker2.cpp)

ament_target_dependencies(target_tracker2
  rclcpp
  std_msgs
  tf2_geometry_msgs
  geometry_msgs
  object_msgs
  visualization_msgs
  
  )

target_link_libraries(target_tracker2
  ${PCL_COMMON_LIBRARIES}
  )

# add_library(${PROJECT_NAME} SHARED
#   target_extractor 
#   #src/target_filter.cpp 
#   #src/target_tracker.cpp
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# install(
#   DIRECTORY include
#   DESTINATION lib/target_extractor
# )

# install(
#   DIRECTORY target_filter
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(
#   DIRECTORY target_tracker
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install binaries
install(TARGETS target_tracker2
  RUNTIME DESTINATION bin
)

install(TARGETS target_tracker2
  DESTINATION lib/${PROJECT_NAME}
)
ament_export_libraries(${library_name})

ament_package()
