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
find_package(GTSAM REQUIRED)
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
 src/Payload.cpp
 src/RobotGTSAM.cpp
 src/DynamicsFactor.cpp
 src/ObstacleFactor.cpp
)
add_executable( test_gtsam
    src/test.cpp
    src/RobotGTSAM.cpp
    src/DynamicsFactor.cpp
    src/ObstacleFactor.cpp
    src/Globals.cpp
    src/Utils.cpp
    src/Simulator.cpp
    src/Graphics.cpp
    src/Payload.cpp
    src/Robot.cpp
)
target_link_libraries(gbpplanner PUBLIC gbp box2d gtsam)
target_link_libraries(test_gtsam PUBLIC gbp box2d gtsam)
target_include_directories(test_gtsam PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)


# Checks if OSX and links appropriate frameworks (only required on MacOS)
if(APPLE)
 foreach(target gbpplanner test_gtsam)
   target_link_libraries(${target} PUBLIC
     "-framework Cocoa"
     "-framework OpenGL"
     "-framework IOKit"
     "-framework CoreVideo"
   )
 endforeach()
endif()
#################################################################