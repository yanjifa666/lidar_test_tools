cmake_minimum_required(VERSION 3.8)
project(sensor_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)

# 包含目录
include_directories(
        include
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(sensor_test src/sensor_test.cpp)

# 配置编译选项
target_compile_features(sensor_test PRIVATE cxx_std_14)
ament_target_dependencies(sensor_test
        rclcpp
        sensor_msgs
        pcl_conversions
)

# 链接库
target_link_libraries(sensor_test
        ${PCL_LIBRARIES}
        OpenMP::OpenMP_CXX

)

# 安装目标
install(TARGETS
        sensor_test
        DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(rclcpp sensor_msgs)
ament_package()
