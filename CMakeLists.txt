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

# 查找现有依赖
find_package(OpenMP REQUIRED)

#################################################################
# MuJoCo配置 - 作为唯一物理引擎
#################################################################

# 设置MuJoCo路径
set(MUJOCO_ROOT "$ENV{HOME}/mujoco_workspace/mujoco-2.3.7" CACHE PATH "MuJoCo installation directory")

# 验证MuJoCo路径
if(NOT EXISTS "${MUJOCO_ROOT}")
    message(FATAL_ERROR "MuJoCo not found at ${MUJOCO_ROOT}. Please install MuJoCo or set correct MUJOCO_ROOT path.")
endif()

message(STATUS "Found MuJoCo at: ${MUJOCO_ROOT}")

# 查找MuJoCo库
find_library(MUJOCO_LIB 
    NAMES mujoco
    PATHS ${MUJOCO_ROOT}/lib
    NO_DEFAULT_PATH
    REQUIRED
)

if(MUJOCO_LIB)
    message(STATUS "Found MuJoCo library: ${MUJOCO_LIB}")
else()
    message(FATAL_ERROR "Could not find MuJoCo library in ${MUJOCO_ROOT}/lib")
endif()

# 查找GLFW (MuJoCo可视化需要)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)

if(GLFW_FOUND)
    message(STATUS "Found GLFW: ${GLFW_LIBRARIES}")
else()
    message(FATAL_ERROR "GLFW is required for MuJoCo visualization")
endif()

if(POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
endif()

# 查找OpenGL
find_package(OpenGL REQUIRED)
if(OpenGL_FOUND)
    message(STATUS "Found OpenGL: ${OPENGL_LIBRARIES}")
endif()

# 设置MuJoCo编译定义
add_compile_definitions(USE_MUJOCO=1)

#################################################################
# Raylib配置
#################################################################
add_subdirectory(thirdparty/raylib)

#################################################################
# GBP库配置
#################################################################
add_library(gbp STATIC
    src/gbp/Variable.cpp
    src/gbp/Factorgraph.cpp
    src/gbp/Factor.cpp
)

target_compile_definitions(gbp PUBLIC 
    ASSETS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/assets/"
    USE_MUJOCO=1
)

# 包含目录
target_include_directories(gbp PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/inc 
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
    ${MUJOCO_ROOT}/include
)

# 链接库
target_link_libraries(gbp PUBLIC 
    OpenMP::OpenMP_CXX 
    raylib
    ${MUJOCO_LIB}
    ${GLFW_LIBRARIES}
    ${OPENGL_LIBRARIES}
)

# 编译选项
target_compile_options(gbp PUBLIC ${GLFW_CFLAGS_OTHER})

#################################################################
# gbpplanner可执行文件配置
#################################################################
add_executable(gbpplanner
    src/main.cpp
    src/Globals.cpp
    src/Robot.cpp
    src/Simulator.cpp
    src/Utils.cpp
    src/Graphics.cpp
    src/Payload.cpp
)

target_include_directories(gbpplanner PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
    ${MUJOCO_ROOT}/include
)

target_link_libraries(gbpplanner PUBLIC 
    gbp
    ${MUJOCO_LIB}
    ${GLFW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    pthread
    dl
)

#################################################################
# 编译选项和平台特定配置
#################################################################

# 为所有目标设置编译选项
foreach(target gbp gbpplanner)
    target_compile_options(${target} PRIVATE
        -Wall
        -Wextra
        -O3
        $<$<CONFIG:Debug>:-g>
        $<$<CONFIG:Debug>:-DDEBUG>
    )
endforeach()

# 平台特定链接库
if(APPLE)
    # macOS特定配置
    foreach(target gbpplanner)
        target_link_libraries(${target} PUBLIC
            "-framework Cocoa"
            "-framework OpenGL"
            "-framework IOKit"
            "-framework CoreVideo"
        )
    endforeach()
elseif(UNIX AND NOT APPLE)
    # Linux特定配置
    foreach(target gbpplanner)
        target_link_libraries(${target} PUBLIC
            X11
            Xrandr
            Xi
            Xcursor
            Xinerama
        )
    endforeach()
elseif(WIN32)
    # Windows特定配置
    foreach(target gbpplanner)
        target_link_libraries(${target} PUBLIC
            gdi32
            opengl32
            winmm
        )
    endforeach()
endif()

#################################################################
# 安装规则
#################################################################
install(TARGETS gbpplanner
    RUNTIME DESTINATION bin
)

# 安装配置文件
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config/
    DESTINATION share/${PROJECT_NAME}/config
    FILES_MATCHING PATTERN "*.json"
)

# 安装资源文件
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/assets/
    DESTINATION share/${PROJECT_NAME}/assets
)

#################################################################
# 构建信息输出
#################################################################
message(STATUS "========================================")
message(STATUS "GBP Planner Build Configuration")
message(STATUS "========================================")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Physics Engine: MuJoCo (only)")
message(STATUS "MuJoCo Root: ${MUJOCO_ROOT}")
message(STATUS "MuJoCo Library: ${MUJOCO_LIB}")
message(STATUS "GLFW Libraries: ${GLFW_LIBRARIES}")
message(STATUS "OpenGL Libraries: ${OPENGL_LIBRARIES}")
message(STATUS "Platform: ${CMAKE_SYSTEM_NAME}")
message(STATUS "========================================")

# 创建构建后的符号链接（可选，方便开发）
if(UNIX)
    add_custom_target(create_symlinks ALL
        COMMAND ${CMAKE_COMMAND} -E create_symlink 
            ${CMAKE_CURRENT_SOURCE_DIR}/config 
            ${CMAKE_CURRENT_BINARY_DIR}/config
        COMMAND ${CMAKE_COMMAND} -E create_symlink 
            ${CMAKE_CURRENT_SOURCE_DIR}/assets 
            ${CMAKE_CURRENT_BINARY_DIR}/assets
        COMMENT "Creating symlinks for config and assets directories"
    )
endif()

# 验证MuJoCo安装的完整性
if(EXISTS "${MUJOCO_ROOT}/include/mujoco/mujoco.h")
    message(STATUS "✓ MuJoCo headers found")
else()
    message(WARNING "✗ MuJoCo headers not found at expected location")
endif()

if(EXISTS "${MUJOCO_LIB}")
    message(STATUS "✓ MuJoCo library verified")
else()
    message(WARNING "✗ MuJoCo library not accessible")
endif()