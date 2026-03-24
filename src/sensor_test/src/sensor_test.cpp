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

// 话题名称定义
const std::string TOPIC_LIDAR_POINT_CLOUD_360 = "/lidar_point_cloud_360";
const std::string TOPIC_LIDAR_POINT_CLOUD_LF = "/lidar_point_left_front";
const std::string TOPIC_LIDAR_POINT_CLOUD_RF = "/lidar_point_right_front";
const std::string TOPIC_LIDAR_POINT_CLOUD_LB = "/lidar_left_back";
const std::string TOPIC_LIDAR_POINT_CLOUD_RB = "/lidar_right_back";

class SensorMerge : public rclcpp::Node
{
public:
    SensorMerge() : Node("pointcloud_merge")
    {
        // 初始化点云指针
        // left_front_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
        // right_front_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
        // left_back_cloud_= std::make_shared<sensor_msgs::msg::PointCloud2>();
        // right_back_cloud_= std::make_shared<sensor_msgs::msg::PointCloud2>();
        last_time_ = this->now();
        last_imu_time_ = this->now();
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/rslidar_imu_data",
                rclcpp::QoS(80000),
                [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                    imuCallback(msg); // 直接调用成员函数
                });


        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
                "imu_base", 10);



        linkhou_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
              "/rslidar_points", rclcpp::QoS(80000),
              [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                  ros2LidarCallBack(msg);
              });

        // right_front_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //         TOPIC_LIDAR_POINT_CLOUD_RF, 10,
        //         std::bind(&SensorMerge::rightFrontCloudCallback, this, std::placeholders::_1));

        // left_back_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //         TOPIC_LIDAR_POINT_CLOUD_LB, rclcpp::SensorDataQoS(),
        //         std::bind(&SensorMerge::leftBackCloudCallback, this, std::placeholders::_1));

        // right_back_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //         TOPIC_LIDAR_POINT_CLOUD_RB, rclcpp::SensorDataQoS(),
        //         std::bind(&SensorMerge::rightBackCloudCallback, this, std::placeholders::_1));
        
        // mid_front_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //         "/lidar_point_mid_front", rclcpp::SensorDataQoS(),
        //         std::bind(&SensorMerge::midFrontCloudCallback, this, std::placeholders::_1));



        // this->declare_parameter<double>("time_sync_threshold", 0.02); // 20毫秒默认阈值
        // time_sync_threshold_ = this->get_parameter("time_sync_threshold").as_double();



        // RCLCPP_INFO(this->get_logger(), "点云转换节点已启动，时间同步阈值: %f秒", time_sync_threshold_);
        // processing_thread_ = std::thread(&SensorMerge::processingThread, this);
    }

private:
    template<typename T>
    Eigen::Matrix<T, 3, 3> EulerToRotM(const Eigen::Matrix<T, 3, 1> &ang)
    {
        Eigen::Matrix<T, 3, 3> R_x, R_y, R_z;
        R_x << 1, 0, 0, 0, cos(ang[0]), -sin(ang[0]), 0, sin(ang[0]), cos(ang[0]);
        R_y << cos(ang[1]), 0, sin(ang[1]), 0, 1, 0, -sin(ang[1]), 0, cos(ang[1]);
        R_z << cos(ang[2]), -sin(ang[2]), 0, sin(ang[2]), cos(ang[2]), 0, 0, 0, 1;
        return R_z * R_y * R_x;
    }


    template<typename T>
    T rad2deg(T radians)
    {

        return radians * 180.0 / M_PI;
    }

    template<typename T>
    T deg2rad(T degrees)
    {
        return degrees * M_PI / 180.0;
    }


    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr& msg)
    {
            // ====== 【新增】时间戳合法性判断 ======
        if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "IMU stamp is ZERO, skip this frame");
            return;
        }
        static int64_t count_ = 0;
        static int64_t count_imu_wran_ =0;
        count_++;

        auto now = this->now();
        auto dt = now - last_imu_time_;

        if (dt.seconds() >= 1.0)
        {   // 每1秒打印一次
            double freq = count_ / dt.seconds();
            // RCLCPP_INFO << "Topic [/livox/lidar] callback frequency: " << freq;
            RCLCPP_INFO(get_logger(),"imu topic frequency:%f",freq);
            count_ = 0;
            last_imu_time_ = now;
        }

        auto current_time = msg->header.stamp.sec * 1e3 + msg->header.stamp.nanosec * 1e-6;

        if (has_last_imu)
        {
            double time_diff_ms = current_time - last_imuPublish_time_;
            if (time_diff_ms > 10.0)
            {
                count_imu_wran_++;
                RCLCPP_WARN(get_logger(),"[%ld] imu前后帧时间差:%f,curr:%f.%f,last:%f.%f",count_imu_wran_,time_diff_ms,msg->header.stamp.sec,
                    msg->header.stamp.nanosec,last_imu_stamp.sec,last_imu_stamp.nanosec);
            }

        }
        else
        {
            has_last_imu = true;
        }
        last_imuPublish_time_ = current_time;
        last_imu_stamp = msg->header.stamp;


        static int i_imu = 0;
        if (i_imu > 600 && i_imu < 800)
        {
            i_imu++;
            return;
        }
        i_imu++;

    }



    void ros2LidarCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr lidarMsg) 
    {
        // ====== 【新增】时间戳合法性判断 ======
        if (lidarMsg->header.stamp.sec == 0 && lidarMsg->header.stamp.nanosec == 0)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "LiDAR stamp is ZERO, skip this frame");
            has_last_pub_time_ = false;
            return;
        }

        static int64_t count_ = 0;
        static int64_t count_warn = 0;
        count_++;

        auto now = this->now();
        auto dt = now - last_time_;

        if (dt.seconds() >= 1.0)
        {   // 每1秒打印一次
            double freq = count_ / dt.seconds();
            // RCLCPP_INFO << "Topic [/livox/lidar] callback frequency: " << freq;
            RCLCPP_INFO(get_logger(),"lidar topic frequency:%f",freq);
            count_ = 0;
            last_time_ = now;
        }

        auto current_time = lidarMsg->header.stamp.sec * 1e3 + lidarMsg->header.stamp.nanosec * 1e-6;
        
        if (has_last_pub_time_)
        {
            double time_diff_ms = current_time - last_publish_time_;
            if (time_diff_ms > 200.0)
            {
                // LOG_S_DEBUG << "前后帧时间差: " << time_diff_ms << "curr: " << lidarMsg->header.stamp.sec
                // << "." << lidarMsg->header.stamp.nanosec << " , last: " << last_stamp.sec <<"." << last_stamp.nanosec;
                count_warn++;
                RCLCPP_WARN(get_logger(),"[%ld] 点云前后帧时间差:%f,curr:%f.%f,last:%f.%f",count_warn,time_diff_ms,lidarMsg->header.stamp.sec,
                    lidarMsg->header.stamp.nanosec,last_stamp.sec,last_stamp.nanosec);
            }
            // LOG_S_DEBUG << "前后帧时间差: " << time_diff_ms << "curr: " << lidarMsg->header.stamp.sec
            //             << "." << lidarMsg->header.stamp.nanosec << " , last: " << last_stamp.sec <<"." << last_stamp.nanosec;
        }
        else
        {
            has_last_pub_time_ = true;
        }

        last_publish_time_ = current_time;
        last_stamp = lidarMsg->header.stamp;


        static int i = 0;
        if (i > 600 && i < 800)
        {
            i++;
            return;
        }
        i++;
    }




private:
    sensor_msgs::msg::PointCloud2::SharedPtr left_front_cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr right_front_cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr left_back_cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr right_back_cloud_;

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
    Eigen::Matrix3d imu_to_base_rot_;

    std::mutex cloud_mutex_;

    bool left_front_received_ = false;
    bool right_front_received_ = false;
    bool left_back_received_ = false;
    bool right_back_received_ = false;

    double time_sync_threshold_ = 0.02; // 20毫秒默认阈值

    bool apply_static_transform_ = true;


    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr linkhou_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::thread processing_thread_;
    std::mutex task_mutex_;
    std::condition_variable task_cv_;
    std::queue<std::function<void()>> task_queue_;
    std::atomic<bool> shutdown_{false};

    bool has_last_pub_time_ = false;
    double last_publish_time_;
    builtin_interfaces::msg::Time last_stamp;
    rclcpp::Time last_time_;

    bool has_last_imu = false;
    double last_imuPublish_time_;
    builtin_interfaces::msg::Time last_imu_stamp;
    rclcpp::Time last_imu_time_;

    void processingThread()
    {
        while (!shutdown_)
        {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(task_mutex_);
                task_cv_.wait(lock, [this]() {
                    return !task_queue_.empty() || shutdown_;
                });

                if (shutdown_ && task_queue_.empty()) break;

                task = std::move(task_queue_.front());
                task_queue_.pop();
            }
            task(); // 执行任务
        }
    }

    template<typename F>
    void postTask(F&& task)
    {
        {
            std::lock_guard<std::mutex> lock(task_mutex_);
            task_queue_.emplace(std::forward<F>(task));
        }
        task_cv_.notify_one();
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorMerge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
