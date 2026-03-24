#include "sensor_trans_merge.h"


SensorMerge::SensorMerge() : Node("pointcloud_merge")
{
    left_front_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    right_front_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    left_back_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    right_back_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    mid_front_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // 添加频率计算相关变量的初始化
    last_publish_time_ = this->now();
    frequency_ = 0.0;
    frame_count_ = 0;
    frequency_calc_start_time_ = this->now();
    frequency_calc_initialized_ = false;

//    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
//            "imu_data", rclcpp::QoS(80000),
//            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
//                imuCallback(msg);
//            });


    merged_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidar_point_cloud_360",
            10
    );


    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_base", 1000);

    lef_front_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            TOPIC_LIDAR_POINT_CLOUD_LF, 10,
            std::bind(&SensorMerge::leftFrontCloudCallback, this, std::placeholders::_1));

    right_front_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            TOPIC_LIDAR_POINT_CLOUD_RF, 10,
            std::bind(&SensorMerge::rightFrontCloudCallback, this, std::placeholders::_1));

    left_back_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            TOPIC_LIDAR_POINT_CLOUD_LB, 10,
            std::bind(&SensorMerge::leftBackCloudCallback, this, std::placeholders::_1));

    right_back_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            TOPIC_LIDAR_POINT_CLOUD_RB, 10,
            std::bind(&SensorMerge::rightBackCloudCallback, this, std::placeholders::_1));
            
  // mid_front_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //         TOPIC_LIDAR_POINT_CLOUD_MD, 10,
  //         std::bind(&SensorMerge::midFrontCloudCallback, this, std::placeholders::_1));

    this->declare_parameter<double>("time_sync_threshold", 0.05);
    time_sync_threshold_ = this->get_parameter("time_sync_threshold").as_double();

    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SensorMerge::fusionAndPublishCallback, this));

    loadStaticTransforms();
    processing_thread_ = std::thread(&SensorMerge::processingThread, this);

    RCLCPP_INFO(this->get_logger(), "点云转换节点已启动，时间同步阈值: %f秒", time_sync_threshold_);
}

SensorMerge::~SensorMerge()
{
    shutdown_.store(true);
    task_cv_.notify_all();
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
}

void SensorMerge::loadStaticTransforms()
{
    left_to_imu_transform_ = Eigen::Matrix4f::Identity();
    imu_to_base_transform_ = Eigen::Matrix4f::Identity();
    right_to_left_transform_ = Eigen::Matrix4f::Identity();
    leftback_to_left_transform_ = Eigen::Matrix4f::Identity();
    rightback_to_base_transform_ = Eigen::Matrix4f::Identity();
    leftback_to_base_transform_ = Eigen::Matrix4f::Identity();
    rightback_to_left_transform_ = Eigen::Matrix4f::Identity();
    mid_to_base_transform_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f mid_to_left_transform_ = Eigen::Matrix4f::Identity();

    //Eigen::Vector3d left_to_imu_rpy(-7.902808, 23.443460, -54.920417);
    // Eigen::Vector3d left_to_imu_rpy(-2.902808, 23.443460, -54.920417);
     Eigen::Vector3d left_to_imu_rpy(-4.646021,  21.442186, -55.354293);
    Eigen::Vector3d imu_to_base_rpy(-176.013, -1.26223, 0.995392);
    //Eigen::Vector3d left_to_imu_rpy(-177.556742, -23.569565 , 23.192771);
    // Eigen::Vector3d imu_to_base_rpy(-176.313, -1.26223, 0.995392);
    // Eigen::Vector3d right_to_left_rpy(-25.591204, 28.311149, 97.460159);
    Eigen::Vector3d right_to_left_rpy(-26.4451, 33.2713, 96.7182);
    Eigen::Vector3d left_to_imu_xyz(0.327, -0.737, -0.459);
    Eigen::Vector3d mid_to_base_rpy(-0.62, -0.57, -0.23);
    Eigen::Vector3d mid_to_base_xyz(1.944, -0.056, 1.425);
     Eigen::Vector3d mid_to_left_rpy(158.152, -18.8339, 58.8572);
    Eigen::Vector3d mid_to_left_xyz(-0.500505, 0.344389 ,-0.458262);

    Eigen::Vector3d imu_to_base_xyz(1.778, 0, 0.71);
    // Eigen::Vector3d right_to_left_xyz(-1.11694, 0.885780, -0.430701);//1 hao
    Eigen::Vector3d right_to_left_xyz(-1.10904 , 0.876854, -0.525592);
    // Eigen::Vector3d leftback_to_left_rpy(6.63787666,  39.54833892,  -111.23790882);//1hao
    //Eigen::Vector3d leftback_to_left_rpy(3.61026, 37.5904, -112.764);
    // Eigen::Vector3d leftback_to_left_rpy(5.53299, 37.0027, -110.15);
    Eigen::Vector3d leftback_to_left_rpy(3.61954, 37.1125, -110.959);
    // Eigen::Vector3d leftback_to_left_xyz( -1.35358095, -2.12082410, -0.89014697);//1hao
    //Eigen::Vector3d leftback_to_left_xyz( -1.4069, -2.05625, -0.790231);
    // Eigen::Vector3d leftback_to_left_xyz( -1.37073 , -2.15894, -0.816175);
    Eigen::Vector3d leftback_to_left_xyz( -1.37313,  -2.13533 ,-0.805673);
    // Eigen::Vector3d rightback_to_leftback_rpy(33.12134947,   3.20588162, 3.79393060);//1 hao
   // Eigen::Vector3d rightback_to_leftback_rpy(42.7096 ,-4.66108 ,-1.10916);
    Eigen::Vector3d rightback_to_leftback_rpy(43.0962 ,-6.81409 ,-1.67097);
    // Eigen::Vector3d rightback_to_leftback_xyz(-0.00984800, -1.29955804,  -0.41457501);//1 hao
   // Eigen::Vector3d rightback_to_leftback_xyz(0.0034163, -1.26914, -0.505306);
    Eigen::Vector3d rightback_to_leftback_xyz(0.0115296, -1.24715, -0.50436);

    left_to_imu_transform_.block<3,3>(0,0) = EulerToRotM(deg2rad(left_to_imu_rpy)).cast<float>();
    imu_to_base_transform_.block<3,3>(0,0) = EulerToRotM(deg2rad(imu_to_base_rpy)).cast<float>();
    right_to_left_transform_.block<3,3>(0,0) = EulerToRotM(deg2rad(right_to_left_rpy)).cast<float>();
    leftback_to_left_transform_.block<3,3>(0,0) = EulerToRotM(deg2rad(leftback_to_left_rpy)).cast<float>();
    rightback_to_left_transform_.block<3,3>(0,0) = EulerToRotM(deg2rad(rightback_to_leftback_rpy)).cast<float>();
    left_to_imu_transform_.block<3,1>(0,3) = left_to_imu_xyz.cast<float>();
    imu_to_base_transform_.block<3,1>(0,3) = imu_to_base_xyz.cast<float>();
    right_to_left_transform_.block<3,1>(0,3) = right_to_left_xyz.cast<float>();
    leftback_to_left_transform_.block<3,1>(0,3) = leftback_to_left_xyz.cast<float>();
    rightback_to_left_transform_.block<3,1>(0,3) = rightback_to_leftback_xyz.cast<float>();
    right_to_base_transform_ = imu_to_base_transform_ * left_to_imu_transform_ * right_to_left_transform_;
    mid_to_base_transform_.block<3,3>(0,0) =EulerToRotM(deg2rad(mid_to_base_rpy)).cast<float>();
    mid_to_base_transform_.block<3,1>(0,3) = mid_to_base_xyz.cast<float>();

    mid_to_left_transform_.block<3,3>(0,0) = EulerToRotM(deg2rad(mid_to_left_rpy)).cast<float>();
    mid_to_left_transform_.block<3,1>(0,3) = mid_to_left_xyz.cast<float>();

    Eigen::Matrix4f offset_transform = Eigen::Matrix4f::Identity();
    // Eigen::Vector3d offset_rpy(-0.11459140, 0.17188759, 0.22918187);
    // Eigen::Vector3d offset_xyz(0.00300000, -0.06600000, 0.01900000);
    Eigen::Vector3d offset_rpy(0,0,0);
    Eigen::Vector3d offset_xyz(0.0000000, -0.000000, 0.000000);
    offset_transform.block<3,3>(0,0) = EulerToRotM(deg2rad(offset_rpy)).cast<float>();
    offset_transform.block<3,1>(0,3) = offset_xyz.cast<float>();

    leftback_to_base_transform_ = offset_transform * imu_to_base_transform_ * left_to_imu_transform_ * leftback_to_left_transform_;
    rightback_to_base_transform_ = leftback_to_base_transform_ * rightback_to_left_transform_;
    left_to_base_transform_ = imu_to_base_transform_ * left_to_imu_transform_;
    imu_to_base_rot_ = EulerToRotM(deg2rad(imu_to_base_rpy));
    mid_to_base_transform_ = imu_to_base_transform_ * left_to_imu_transform_ * mid_to_left_transform_;

    RCLCPP_INFO(this->get_logger(), "静态变换矩阵加载完成");
}

void SensorMerge::transformImu(const sensor_msgs::msg::Imu::SharedPtr& msg)
{
    static  double last_timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    double cur_timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    double delt_tm = cur_timestamp - last_timestamp;
    if(delt_tm > 0.1)
        RCLCPP_INFO(this->get_logger(),"delt time = %ld", delt_tm);


    sensor_msgs::msg::Imu::SharedPtr msg_tran(new sensor_msgs::msg::Imu(*msg));
    Eigen::Vector3d ang(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    ang = imu_to_base_rot_ * ang;
    Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    acc = imu_to_base_rot_ * acc;

    msg_tran->angular_velocity.x = ang.x();
    msg_tran->angular_velocity.y = ang.y();
    msg_tran->angular_velocity.z = ang.z();
    msg_tran->linear_acceleration.x = acc.x();
    msg_tran->linear_acceleration.y = acc.y();
    msg_tran->linear_acceleration.z = acc.z();

    imu_pub_->publish(*msg_tran);
}


void SensorMerge::transformPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& input,
                                      sensor_msgs::msg::PointCloud2::SharedPtr& output,
                                      const Eigen::Matrix4f& transform)
{
    if (input->data.empty()) return;
    *output = *input;

    // 预计算字段偏移量
    int x_offset = -1, y_offset = -1, z_offset = -1;
    for (const auto& field : input->fields) {
        if (field.name == "x") x_offset = field.offset;
        else if (field.name == "y") y_offset = field.offset;
        else if (field.name == "z") z_offset = field.offset;
    }

    if (x_offset == -1 || y_offset == -1 || z_offset == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("transform"), "Missing coordinate fields");
        return;
    }

    const uint32_t point_step = input->point_step;
    const uint32_t num_points = input->width * input->height;
    const uint8_t* in_data = input->data.data();
    uint8_t* out_data = output->data.data();
    const Eigen::Matrix4f& T = transform;
    const float* T_data = T.data();

    std::vector<size_t> index(num_points);
    for (size_t i = 0; i < num_points; ++i)
    {
        index[i] = i;
    }

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i)
    {
        const uint8_t* in_point = in_data + i * point_step;
        uint8_t* out_point = out_data + i * point_step;
        float x, y, z;
        std::memcpy(&x, in_point + x_offset, sizeof(float));
        std::memcpy(&y, in_point + y_offset, sizeof(float));
        std::memcpy(&z, in_point + z_offset, sizeof(float));
        float tx = T_data[0] * x + T_data[4] * y + T_data[8] * z + T_data[12];
        float ty = T_data[1] * x + T_data[5] * y + T_data[9] * z + T_data[13];
        float tz = T_data[2] * x + T_data[6] * y + T_data[10] * z + T_data[14];
        std::memcpy(out_point + x_offset, &tx, sizeof(float));
        std::memcpy(out_point + y_offset, &ty, sizeof(float));
        std::memcpy(out_point + z_offset, &tz, sizeof(float));
    });


}


void SensorMerge::imuCallback(const sensor_msgs::msg::Imu::SharedPtr& msg)
{
    transformImu(msg);
}

void SensorMerge::leftFrontCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    addPointCloudTask(msg, left_to_base_transform_, "left_front");
}

void SensorMerge::rightFrontCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    addPointCloudTask(msg, right_to_base_transform_, "right_front");
}

void SensorMerge::leftBackCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    addPointCloudTask(msg, leftback_to_base_transform_, "left_back");
}

void SensorMerge::rightBackCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    addPointCloudTask(msg, rightback_to_base_transform_, "right_back");
}

void SensorMerge::midFrontCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    addPointCloudTask(msg, mid_to_base_transform_, "mid_front");
}

void SensorMerge::addPointCloudTask(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                                    const Eigen::Matrix4f& transform, const std::string& cloud_type)
{
    postTask([this, msg, transform, cloud_type]()
             {
                 sensor_msgs::msg::PointCloud2::SharedPtr output_cloud =
                         std::make_shared<sensor_msgs::msg::PointCloud2>();

                 transformPointCloud(msg, output_cloud, transform);

                 std::lock_guard<std::mutex> lock(cloud_mutex_);

                 if (cloud_type == "left_front")
                 {
                     left_front_cloud_ = output_cloud;
                     left_front_stamp_ = msg->header.stamp;
                     left_front_received_ = true;
                 }
                 else if (cloud_type == "right_front")
                 {
                     right_front_cloud_ = output_cloud;
                     right_front_stamp_ = msg->header.stamp;
                     right_front_received_ = true;
                 }
                 else if (cloud_type == "left_back")
                 {
                     left_back_cloud_ = output_cloud;
                     left_back_stamp_ = msg->header.stamp;
                     left_back_received_ = true;
                 }
                 else if (cloud_type == "right_back")
                 {
                     right_back_cloud_ = output_cloud;
                     right_back_stamp_ = msg->header.stamp;
                     right_back_received_ = true;
                 }
                else if (cloud_type == "mid_front")
                 {
                     mid_front_cloud_ = output_cloud;
                     mid_front_stamp_ = msg->header.stamp;
                     mid_front_received_ = true;
                 }
             });
}


bool SensorMerge::allRequiredCloudsReceived()
{
    return left_front_received_ && left_back_received_
           &&right_front_received_&&right_back_received_;
}

void SensorMerge::resetReceiveStatus()
{
    left_front_received_ = false;
    right_front_received_ = false;
    left_back_received_ = false;
    right_back_received_ = false;
    mid_front_received_ = false;

    left_front_syn_ = true;
    right_front_syn_ = true;
    left_back_syn_ = true;
    right_back_syn_ = true;
    mid_front_syn_ = true;
}
/*
bool SensorMerge::getLatestTimestamp(const builtin_interfaces::msg::Time& left_front_stamp,
                                     const builtin_interfaces::msg::Time& right_front_stamp,
                                     const builtin_interfaces::msg::Time& left_back_stamp,
                                     const builtin_interfaces::msg::Time& right_back_stamp,
                                     builtin_interfaces::msg::Time& merge_time)
{
    std::vector<builtin_interfaces::msg::Time> v_ts;
    v_ts.push_back(left_front_stamp);
    v_ts.push_back(right_front_stamp);
    v_ts.push_back(left_back_stamp);
    v_ts.push_back(right_back_stamp);
    std::vector<std::pair<double, builtin_interfaces::msg::Time>> timestamps;
    for (int i = 0; i < v_ts.size(); i++)
    {
        double time = v_ts[i].sec + v_ts[i].nanosec * 1e-9;
        if (time != 0.0)
        {
            timestamps.push_back({v_ts[i].sec + v_ts[i].nanosec * 1e-9, v_ts[i]});
        }
    }

    if (timestamps.empty()) return false;

    auto comp = [](const auto& a, const auto& b) {
        return a.first < b.first;
    };

    auto max_it = std::max_element(timestamps.begin(), timestamps.end(), comp);
    double max_time = max_it->second.sec + max_it->second.nanosec * 1e-9;
    double time_thr = max_time - time_sync_threshold_;


    if((left_front_stamp.sec + left_front_stamp.nanosec * 1e-9) < time_thr)
    {
        left_front_syn_ = false;
        RCLCPP_WARN(this->get_logger(), "left front pop out");
    }

    if((right_front_stamp.sec + right_front_stamp.nanosec * 1e-9) < time_thr)
    {
        right_front_syn_ = false;
        RCLCPP_WARN(this->get_logger(), "right front pop out");
    }

    if((left_back_stamp.sec + left_back_stamp.nanosec * 1e-9) < time_thr)
    {
        left_back_syn_ = false;
        RCLCPP_WARN(this->get_logger(), "left back pop out");
    }

    if((right_back_stamp.sec + right_back_stamp.nanosec * 1e-9) < time_thr)
    {
        right_back_syn_ = false;
        RCLCPP_WARN(this->get_logger(), "right back pop out");
    }

//    auto min_it = std::min_element(timestamps.begin(), timestamps.end(), comp);
//    // TODO: if diff > 20ms , dont push
//
//    double max_time = max_it->second.sec + max_it->second.nanosec * 1e-9;
//    double min_time = min_it->second.sec + min_it->second.nanosec * 1e-9;
//    double time_diff = std::abs(max_time - min_time);
//
//    if (time_diff > time_sync_threshold_)
//    {
//        RCLCPP_WARN(this->get_logger(),
//                    "雷达时间戳差异过大: %.3f秒 (阈值: %.3f秒)，跳过本次融合",
//                    time_diff, time_sync_threshold_);
//        return false;
//    }

    merge_time = max_it->second;
    return true;
}
*/

bool SensorMerge::getLatestTimestamp(const builtin_interfaces::msg::Time& left_front_stamp,
                                     const builtin_interfaces::msg::Time& right_front_stamp,
                                     const builtin_interfaces::msg::Time& left_back_stamp,
                                     const builtin_interfaces::msg::Time& right_back_stamp,
                                     builtin_interfaces::msg::Time& merge_time)
{
    std::vector<builtin_interfaces::msg::Time> v_ts;
    v_ts.push_back(left_front_stamp);
    v_ts.push_back(right_front_stamp);
    v_ts.push_back(left_back_stamp);
    v_ts.push_back(right_back_stamp);
    std::vector<std::pair<double, builtin_interfaces::msg::Time>> timestamps;
    for (int i = 0; i < v_ts.size(); i++)
    {
        double time = v_ts[i].sec + v_ts[i].nanosec * 1e-9;
        if (time != 0.0)
        {
            timestamps.push_back({v_ts[i].sec + v_ts[i].nanosec * 1e-9, v_ts[i]});
        }
    }

    if (timestamps.empty()) return false;

    auto comp = [](const auto& a, const auto& b) {
        return a.first < b.first;
    };

//    auto max_it = std::max_element(timestamps.begin(), timestamps.end(), comp);
//    double max_time = max_it->second.sec + max_it->second.nanosec * 1e-9;

    double max_time = left_front_stamp.sec * 1e3 + left_front_stamp.nanosec * 1e-6;

    if(abs(right_front_stamp.sec * 1e3 + right_front_stamp.nanosec * 1e-6 - max_time) > 50)
    {
        right_front_syn_ = false;
        RCLCPP_WARN(this->get_logger(), "right front pop out ,时间戳: %d.%09d", right_front_stamp.sec, right_front_stamp.nanosec);
    }

    if(abs(left_back_stamp.sec * 1e3 + left_back_stamp.nanosec * 1e-6 - max_time) > 50)
    {
        left_back_syn_ = false;
        RCLCPP_WARN(this->get_logger(), "left back pop out", left_back_stamp.sec, left_back_stamp.nanosec);
    }

    if(abs(right_back_stamp.sec * 1e3 + right_back_stamp.nanosec * 1e-6 - max_time) > 50)
    {
        right_back_syn_ = false;
        RCLCPP_WARN(this->get_logger(), "right back pop out", right_back_stamp.sec, right_back_stamp.nanosec);
    }

//    auto min_it = std::min_element(timestamps.begin(), timestamps.end(), comp);
//    // TODO: if diff > 20ms , dont push
//
//    double max_time = max_it->second.sec + max_it->second.nanosec * 1e-9;
//    double min_time = min_it->second.sec + min_it->second.nanosec * 1e-9;
//    double time_diff = std::abs(max_time - min_time);
//
//    if (time_diff > time_sync_threshold_)
//    {
//        RCLCPP_WARN(this->get_logger(),
//                    "雷达时间戳差异过大: %.3f秒 (阈值: %.3f秒)，跳过本次融合",
//                    time_diff, time_sync_threshold_);
//        return false;
//    }

    merge_time = left_front_stamp;
    return true;
}


void SensorMerge::fusionAndPublishCallback()
{
    std::lock_guard<std::mutex> lock(cloud_mutex_);

    // if (!allRequiredCloudsReceived())
    // {
    //     return;
    // }



    double merge_begin = omp_get_wtime();

    builtin_interfaces::msg::Time merged_stamp;
   //  if (!getLatestTimestamp(left_front_stamp_, right_front_stamp_,
   //                          left_back_stamp_, right_back_stamp_, merged_stamp))
  //   {
   //      return;
   //  }

    size_t total_points = 0;
    if (left_front_syn_) total_points += left_front_cloud_->width * left_front_cloud_->height;
    if (right_front_syn_) total_points += right_front_cloud_->width * right_front_cloud_->height;
    if (left_back_syn_) total_points += left_back_cloud_->width * left_back_cloud_->height;
    if (right_back_syn_) total_points += right_back_cloud_->width * right_back_cloud_->height;
    if (mid_front_syn_) total_points += mid_front_cloud_->width * mid_front_cloud_->height;

    if (total_points == 0)
    {
        RCLCPP_WARN(this->get_logger(), "所有点云都为空，跳过发布");
        resetReceiveStatus();
        return;
    }

    sensor_msgs::msg::PointCloud2::SharedPtr merged_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

    if (left_front_received_ && !left_front_cloud_->data.empty())
    {
        merged_cloud->header = left_front_cloud_->header;
        merged_cloud->point_step = left_front_cloud_->point_step;
        merged_cloud->fields = left_front_cloud_->fields;
        merged_cloud->is_bigendian = left_front_cloud_->is_bigendian;
        merged_cloud->is_dense = left_front_cloud_->is_dense;
    }
    else if (right_front_received_ && !right_front_cloud_->data.empty())
    {
        merged_cloud->header = right_front_cloud_->header;
        merged_cloud->point_step = right_front_cloud_->point_step;
        merged_cloud->fields = right_front_cloud_->fields;
        merged_cloud->is_bigendian = right_front_cloud_->is_bigendian;
        merged_cloud->is_dense = right_front_cloud_->is_dense;
    }
    else if (left_back_received_ && !left_back_cloud_->data.empty())
    {
        merged_cloud->header = left_back_cloud_->header;
        merged_cloud->point_step = left_back_cloud_->point_step;
        merged_cloud->fields = left_back_cloud_->fields;
        merged_cloud->is_bigendian = left_back_cloud_->is_bigendian;
        merged_cloud->is_dense = left_back_cloud_->is_dense;
    }
    else if(right_back_received_ && !right_back_cloud_->data.empty())
    {
        merged_cloud->header = right_back_cloud_->header;
        merged_cloud->point_step = right_back_cloud_->point_step;
        merged_cloud->fields = right_back_cloud_->fields;
        merged_cloud->is_bigendian = right_back_cloud_->is_bigendian;
        merged_cloud->is_dense = right_back_cloud_->is_dense;
    }
    else
    {
        merged_cloud->header = mid_front_cloud_->header;
        merged_cloud->point_step = mid_front_cloud_->point_step;
        merged_cloud->fields = mid_front_cloud_->fields;
        merged_cloud->is_bigendian = mid_front_cloud_->is_bigendian;
        merged_cloud->is_dense = mid_front_cloud_->is_dense;
    }

    merged_cloud->data.clear();
    merged_cloud->width = total_points;
    merged_cloud->height = 1;
    merged_cloud->row_step = merged_cloud->width * merged_cloud->point_step;
    merged_cloud->data.resize(merged_cloud->row_step * merged_cloud->height);

    size_t current_offset = 0;
    auto appendCloud = [&](const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
        if (!cloud || cloud->data.empty())
            return;

        size_t cloud_points = cloud->width * cloud->height;
        size_t data_size = cloud_points * cloud->point_step;

        if (current_offset + data_size <= merged_cloud->data.size())
        {
            memcpy(&merged_cloud->data[current_offset], cloud->data.data(), data_size);
            current_offset += data_size;
        }
    };

    // 按顺序添加点云
    if (left_front_syn_) appendCloud(left_front_cloud_);
    if (right_front_syn_) appendCloud(right_front_cloud_);
    if (left_back_syn_) appendCloud(left_back_cloud_);
    if (right_back_syn_) appendCloud(right_back_cloud_);
   if (mid_front_syn_) appendCloud(mid_front_cloud_);

//    pcl::shared_ptr<HgPointCloudXYZRTLT> pclPointCloud(new HgPointCloudXYZRTLT());
//    pcl::moveFromROSMsg(*right_back_cloud_, *pclPointCloud);
//    if(!pclPointCloud->empty())
//       pcl::io::savePCDFileBinary("./right_backt_cloud.pcd", *pclPointCloud);
//    static bool init = false;
//    if(init)
//        return;
//   pcl::shared_ptr<HgPointCloudXYZRTLT> AllpclPointCloud(new HgPointCloudXYZRTLT());
//   pcl::moveFromROSMsg(*merged_cloud, *AllpclPointCloud);
//   if(!AllpclPointCloud->empty())
//   {
//       pcl::io::savePCDFileBinary("./merged_cloud-all.pcd", *AllpclPointCloud);
//     //    init = true;
//   }

    // 设置时间戳和坐标系
    merged_cloud->header.stamp = merged_stamp;
    merged_cloud->header.frame_id = "base_link";

    // 发布点云
    merged_cloud_pub_->publish(*merged_cloud);

    // 计算发布频率
    auto current_time = this->now();

    double time_interval = (current_time - last_publish_time_).seconds();
    double instant_frequency = (time_interval > 0) ? 1.0 / time_interval : 0.0;

    frame_count_++;
    double elapsed_time = (current_time - frequency_calc_start_time_).seconds();
    double average_frequency = (elapsed_time > 0) ? frame_count_ / elapsed_time : 0.0;

    // 每N帧重置一次平均频率计算，避免长期累积误差, 每100帧重置一次
    if (frame_count_ >= 100)
    {
        frame_count_ = 0;
        frequency_calc_start_time_ = current_time;
    }

    double merge_time = omp_get_wtime() - merge_begin;

    if (has_last_pub_time_)
    {
        // 计算时间差（先转为纳秒，再除以1e6得到毫秒）
        int64_t time_diff_ms = (current_time - last_publish_time_).nanoseconds() / 1000000;
        // 打印毫秒级时间差超过200ms
        if (time_diff_ms > 200)
        {
            RCLCPP_INFO(this->get_logger(),
                    "\033[36m[前后帧时间差] publishClusterObjects: %lldms\033[0m", time_diff_ms);
            RCLCPP_INFO(this->get_logger(),
                        "发布融合点云，点数: %zu, 时间戳: %d.%09d, 消耗: %.6f秒, 瞬时频率: %.2f Hz, 平均频率: %.2f Hz",
                        total_points, merged_stamp.sec, merged_stamp.nanosec, merge_time,
                        instant_frequency, average_frequency);
        }
    }
    else
    {
        has_last_pub_time_ = true;
    }


    // 更新上次发布的时间戳
    last_publish_time_ = current_time;

//    RCLCPP_INFO(this->get_logger(),
//                "发布融合点云，点数: %zu, 时间戳: %d.%09d, 消耗: %.6f秒, 瞬时频率: %.2f Hz, 平均频率: %.2f Hz",
//                total_points, merged_stamp.sec, merged_stamp.nanosec, merge_time,
//                instant_frequency, average_frequency);

    resetReceiveStatus();
}

void SensorMerge::processingThread()
{
    while (!shutdown_.load()) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(task_mutex_);
            task_cv_.wait(lock, [this]() {
                return !task_queue_.empty() || shutdown_.load();
            });

            if (shutdown_.load() && task_queue_.empty()) break;

            if (!task_queue_.empty()) {
                task = std::move(task_queue_.front());
                task_queue_.pop();
            }
        }
        if (task) task();
    }
}

template<typename F>
void SensorMerge::postTask(F&& task)
{
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        task_queue_.emplace(std::forward<F>(task));
    }
    task_cv_.notify_one();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorMerge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
