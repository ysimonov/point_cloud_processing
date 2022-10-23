cmake_minimum_required(VERSION 3.6 FATAL_ERROR)
project(object_detection)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compile options
add_compile_options("-O2" "-Wall" "-funroll-loops" "-lpthread")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

# include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
)

file(GLOB SOURCES
    "${PROJECT_SOURCE_DIR}/include/*.h"
    "${PROJECT_SOURCE_DIR}/include/*.hpp"
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
    "${PROJECT_SOURCE_DIR}/src/*.c"
)

# Eigen
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# VTK
find_package(VTK 9.2 REQUIRED)

# PCL
find_package(PCL 1.7.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# executable and build
add_executable(object_detection main.cpp ${SOURCES})

# linking
target_link_libraries(object_detection Eigen3::Eigen)
target_link_libraries(object_detection ${VTK_LIBRARIES})
target_link_libraries(object_detection ${PCL_LIBRARIES})


# ament_target_dependencies(publisher_lambda rclcpp std_msgs) 

# ament_package()

# install(TARGETS 
#    publisher_lambda
#    DESTINATION lib/${PROJECT_NAME}
# )