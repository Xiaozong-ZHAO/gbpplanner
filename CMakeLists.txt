cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
# Set default build type to release
if(NOT CMAKE_BUILD_TYPE)
 set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()
# Name of Project
project(GBPPlanner)
add_compile_definitions(EIGEN_DONT_PARALLELIZE)
find_package(OpenMP REQUIRED)
find_package(box2d REQUIRED)
add_subdirectory(thirdparty/raylib)
#################################################################
add_library(gbp STATIC
 src/gbp/Variable.cpp
 src/gbp/Factorgraph.cpp
 src/gbp/Factor.cpp
)
target_compile_definitions(gbp PUBLIC ASSETS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/assets/")
target_include_directories(gbp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
target_link_libraries(gbp PUBLIC OpenMP::OpenMP_CXX raylib)

add_executable(gbpplanner
 src/main.cpp
 src/Globals.cpp
 src/Robot.cpp
 src/Simulator.cpp
 src/Utils.cpp
 src/Graphics.cpp
)
target_link_libraries(gbpplanner PUBLIC gbp box2d)

# 先定义目标，再链接库
add_executable(payload_simulator
 src/Payload.cpp
 src/Globals.cpp
 src/Robot.cpp
 src/Simulator.cpp
 src/Utils.cpp
 src/Graphics.cpp
)

# 包含头文件目录
target_include_directories(payload_simulator PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}/inc
${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
)

# 链接库
target_link_libraries(payload_simulator PUBLIC
    gbp
 raylib
 box2d
 OpenMP::OpenMP_CXX
)

# 移除了错误的链接行，因为已经在上面正确链接了 box2d
# 不需要：target_link_libraries(payload_simulator PUBLIC Box2D::Box2D)

# Checks if OSX and links appropriate frameworks (only required on MacOS)
if(APPLE)
 foreach(target gbpplanner payload_simulator)
 target_link_libraries(${target} PUBLIC
"-framework Cocoa"
"-framework OpenGL"
"-framework IOKit"
"-framework CoreVideo"
 )
 endforeach()
endif()
#################################################################