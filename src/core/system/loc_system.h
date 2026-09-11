//
// Created by xiang on 25-9-8.
//

#ifndef LIGHTNING_LOC_SYSTEM_H
#define LIGHTNING_LOC_SYSTEM_H

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"
#include "core/localization/localization_result.h"

namespace lightning {

namespace loc {
class Localization;
}

class LocSystem {
   public:
    struct Options {
        bool pub_tf_ = true;  // 是否发布tf
    };

    explicit LocSystem(Options options);
    ~LocSystem();

    /// 初始化，地图路径在yaml里配置
    bool Init(const std::string& yaml_path);

    /// 设置初始化位姿
    void SetInitPose(const SE3& pose);

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// 处理点云
    void ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
    void ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud);

    /// 实时模式下的spin
    void Spin();

   private:
    void HandleLocalizationResult(const loc::LocalizationResult& result);
    void HandleInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message);

    Options options_;

    std::shared_ptr<loc::Localization> loc_ = nullptr;  // 定位接口

    std::atomic_bool loc_started_ = false;  // 是否开启定位
    std::atomic_bool map_loaded_ = false;   // 地图是否已载入

    /// 实时模式下的ros2 node, subscribers
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;

    std::string map_frame_ = "map";
    std::string odom_frame_ = "odom";
    std::string base_frame_ = "base_link";

    std::string imu_topic_;
    std::string cloud_topic_;
    std::string livox_topic_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_ = nullptr;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_ = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_ = nullptr;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr localization_status_pub_ = nullptr;
};

};  // namespace lightning

#endif  // LIGHTNING_LOC_SYSTEM_H
