#ifndef POINTCLOUD_MERGE_HPP
#define POINTCLOUD_MERGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <deque>
#include <mutex>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <queue>
#include <thread>
#include <condition_variable>
#include <memory>
#include <atomic>
#include <omp.h>
#include <execution>


// 话题名称定义
const std::string TOPIC_LIDAR_POINT_CLOUD_360 = "/lidar_point_cloud_360";
const std::string TOPIC_LIDAR_POINT_CLOUD_LF = "/lidar_point_left_front";
const std::string TOPIC_LIDAR_POINT_CLOUD_RF = "/lidar_point_right_front";
const std::string TOPIC_LIDAR_POINT_CLOUD_LB = "/lidar_point_left_back";
const std::string TOPIC_LIDAR_POINT_CLOUD_RB = "/lidar_point_right_back";
const std::string TOPIC_LIDAR_POINT_CLOUD_MD = "/lidar_point_mid_front";

struct EIGEN_ALIGN16 HgPointXYZRTLT
{
    PCL_ADD_POINT4D;
    float intensity;
    unsigned char tag;
    unsigned char line;
    double  timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(HgPointXYZRTLT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (unsigned char, tag, tag)
    (unsigned char, line, line)
    (double, timestamp, timestamp))
using HgPointCloudXYZRTLT = pcl::PointCloud<HgPointXYZRTLT>;

class SensorMerge : public rclcpp::Node
{
public:
    SensorMerge();
    ~SensorMerge();

private:
    // 模板函数声明
    template<typename T>
    Eigen::Matrix<T, 3, 3> EulerToRotM(const Eigen::Matrix<T, 3, 1> &ang);

    template<typename T>
    T rad2deg(T radians);

    template<typename T>
    T deg2rad(T degrees);

    // 成员函数声明
    void loadStaticTransforms();
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr& msg);
    void transformImu(const sensor_msgs::msg::Imu::SharedPtr& msg);
    void transformPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& input,
                             sensor_msgs::msg::PointCloud2::SharedPtr& output,
                             const Eigen::Matrix4f& transform);

    void leftFrontCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void rightFrontCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void leftBackCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void rightBackCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void midFrontCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    bool allRequiredCloudsReceived();
    void resetReceiveStatus();

    bool getLatestTimestamp(const builtin_interfaces::msg::Time& left_front_stamp,
                                         const builtin_interfaces::msg::Time& right_front_stamp,
                                         const builtin_interfaces::msg::Time& left_back_stamp,
                                         const builtin_interfaces::msg::Time& right_back_stamp,
                                         builtin_interfaces::msg::Time& merge_time);
    void fusionAndPublishCallback();

    void processingThread();
    template<typename F>
    void postTask(F&& task);
    void addPointCloudTask(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                                        const Eigen::Matrix4f& transform, const std::string& cloud_type);
    // 成员变量
    sensor_msgs::msg::PointCloud2::SharedPtr left_front_cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr right_front_cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr left_back_cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr right_back_cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr mid_front_cloud_;

    builtin_interfaces::msg::Time left_front_stamp_;
    builtin_interfaces::msg::Time right_front_stamp_;
    builtin_interfaces::msg::Time left_back_stamp_;
    builtin_interfaces::msg::Time right_back_stamp_;
    builtin_interfaces::msg::Time mid_front_stamp_;

    Eigen::Matrix4f left_to_imu_transform_;
    Eigen::Matrix4f imu_to_base_transform_;
    Eigen::Matrix4f right_to_left_transform_;
    Eigen::Matrix4f right_to_base_transform_;
    Eigen::Matrix4f left_to_base_transform_;
    Eigen::Matrix4f mid_to_base_transform_;
    Eigen::Matrix3d imu_to_base_rot_;
    Eigen::Matrix4f leftback_to_left_transform_;
    Eigen::Matrix4f leftback_to_base_transform_;
    Eigen::Matrix4f rightback_to_left_transform_;
    Eigen::Matrix4f rightback_to_base_transform_;

    std::mutex cloud_mutex_;

    bool left_front_received_ = false;
    bool right_front_received_ = false;
    bool left_back_received_ = false;
    bool right_back_received_ = false;
    bool mid_front_received_ = false;
    

    bool left_front_syn_ = true;
    bool right_front_syn_ = true;
    bool left_back_syn_ = true;
    bool right_back_syn_ = true;
    bool mid_front_syn_ = true;

    double time_sync_threshold_ = 0.02;

    bool apply_static_transform_ = true;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_cloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lef_front_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_front_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_back_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_back_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mid_front_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::thread processing_thread_;
    std::mutex task_mutex_;
    std::condition_variable task_cv_;
    std::queue<std::function<void()>> task_queue_;
    std::atomic<bool> shutdown_{false};

    rclcpp::Time last_publish_time_;
    double frequency_;
    size_t frame_count_;
    rclcpp::Time frequency_calc_start_time_;
    bool frequency_calc_initialized_;
    bool has_last_pub_time_ = false;
};

// 模板函数实现（必须在头文件中）
template<typename T>
Eigen::Matrix<T, 3, 3> SensorMerge::EulerToRotM(const Eigen::Matrix<T, 3, 1> &ang)
{
    Eigen::Matrix<T, 3, 3> R_x, R_y, R_z;
    R_x << 1, 0, 0,
            0, cos(ang[0]), -sin(ang[0]),
            0, sin(ang[0]), cos(ang[0]);
    R_y << cos(ang[1]), 0, sin(ang[1]),
            0, 1, 0,
            -sin(ang[1]), 0, cos(ang[1]);
    R_z << cos(ang[2]), -sin(ang[2]), 0,
            sin(ang[2]), cos(ang[2]), 0,
            0, 0, 1;
    return R_z * R_y * R_x;
}

template<typename T>
T SensorMerge::rad2deg(T radians)
{
    return radians * 180.0 / M_PI;
}

template<typename T>
T SensorMerge::deg2rad(T degrees)
{
    return degrees * M_PI / 180.0;
}

#endif // POINTCLOUD_MERGE_HPP
