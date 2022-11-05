cmake_minimum_required(VERSION 3.6 FATAL_ERROR)
project(object_detection)

# Default C Standard
set(CMAKE_C_STANDARD 11)

# Default C++ Standard
set(CMAKE_CXX_STANDARD 14)

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
    ${PROJECT_SOURCE_DIR}/patchworkpp/include
    ${PROJECT_SOURCE_DIR}/patchworkpp/src
)

file(GLOB SOURCES
    "${PROJECT_SOURCE_DIR}/include/*.h"
    "${PROJECT_SOURCE_DIR}/include/*.hpp"
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
    "${PROJECT_SOURCE_DIR}/src/*.c"
    "${PROJECT_SOURCE_DIR}/patchworkpp/include/*.hpp"
    "${PROJECT_SOURCE_DIR}/patchworkpp/src/*.cpp"
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

# OpenMP
# find_package(OpenMP)
# if (OPENMP_FOUND)
#     set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#     set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#     set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
# endif()

add_subdirectory(patchworkpp)

# executable and build
add_executable(object_detection main.cpp ${SOURCES})

# linking
target_link_libraries(object_detection PATCHWORK::patchworkpp)
target_link_libraries(object_detection Eigen3::Eigen)
target_link_libraries(object_detection ${VTK_LIBRARIES})
target_link_libraries(object_detection ${PCL_LIBRARIES})


# ament_target_dependencies(publisher_lambda rclcpp std_msgs) 

# ament_package()

# install(TARGETS 
#    publisher_lambda
#    DESTINATION lib/${PROJECT_NAME}
# )