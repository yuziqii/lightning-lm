//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"
#include "core/localization/localization.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"

#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>

namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() { loc_->Finish(); }

bool LocSystem::Init(const std::string &yaml_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);

    std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    LOG(INFO) << "online mode, creating ros2 node ... ";

    /// subscribers
    node_ = std::make_shared<rclcpp::Node>("lightning_localization");

    map_frame_ = node_->declare_parameter<std::string>("map_frame", map_frame_);
    odom_frame_ = node_->declare_parameter<std::string>("odom_frame", odom_frame_);
    base_frame_ = node_->declare_parameter<std::string>("base_frame", base_frame_);

    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");

    rclcpp::QoS qos(10);

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

            ProcessIMU(imu);
        });

    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    initial_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", qos, [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
            HandleInitialPose(message);
        });

    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/lightning/pose", qos);
    localization_status_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/lightning/localization_status", qos);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (options_.pub_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    }

    loc_->SetLocalizationResultCallback(
        [this](const loc::LocalizationResult& result) { HandleLocalizationResult(result); });

    bool ret = loc_->Init(yaml_path, map_path);
    if (ret) {
        LOG(INFO) << "online loc node has been created.";
    }

    return ret;
}

void LocSystem::SetInitPose(const SE3 &pose) {
    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();

    loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
    loc_started_ = true;
}

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    if (loc_started_) {
        loc_->ProcessIMUMsg(imu);
    }
}

void LocSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLidarMsg(cloud);
    }
}

void LocSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLivoxLidarMsg(cloud);
    }
}

void LocSystem::Spin() {
    if (node_ != nullptr) {
        spin(node_);
    }
}

void LocSystem::HandleLocalizationResult(const loc::LocalizationResult& result) {
    std_msgs::msg::Int32 status;
    status.data = static_cast<int>(result.status_);
    localization_status_pub_->publish(status);

    if (!result.valid_) {
        return;
    }

    const auto map_to_base = result.ToGeoMsg();
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header = map_to_base.header;
    pose.header.frame_id = map_frame_;
    pose.pose.pose.position.x = map_to_base.transform.translation.x;
    pose.pose.pose.position.y = map_to_base.transform.translation.y;
    pose.pose.pose.position.z = map_to_base.transform.translation.z;
    pose.pose.pose.orientation = map_to_base.transform.rotation;
    pose_pub_->publish(pose);

    if (!tf_broadcaster_) {
        return;
    }

    try {
        const auto odom_to_base = tf_buffer_->lookupTransform(
            odom_frame_, base_frame_, rclcpp::Time(map_to_base.header.stamp), tf2::durationFromSec(0.05));

        const tf2::Transform map_base(
            tf2::Quaternion(map_to_base.transform.rotation.x, map_to_base.transform.rotation.y,
                            map_to_base.transform.rotation.z, map_to_base.transform.rotation.w),
            tf2::Vector3(map_to_base.transform.translation.x, map_to_base.transform.translation.y,
                         map_to_base.transform.translation.z));
        const tf2::Transform odom_base(
            tf2::Quaternion(odom_to_base.transform.rotation.x, odom_to_base.transform.rotation.y,
                            odom_to_base.transform.rotation.z, odom_to_base.transform.rotation.w),
            tf2::Vector3(odom_to_base.transform.translation.x, odom_to_base.transform.translation.y,
                         odom_to_base.transform.translation.z));
        const tf2::Transform map_odom = map_base * odom_base.inverse();

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = map_to_base.header.stamp;
        transform.header.frame_id = map_frame_;
        transform.child_frame_id = odom_frame_;
        transform.transform.translation.x = map_odom.getOrigin().x();
        transform.transform.translation.y = map_odom.getOrigin().y();
        transform.transform.translation.z = map_odom.getOrigin().z();
        transform.transform.rotation.x = map_odom.getRotation().x();
        transform.transform.rotation.y = map_odom.getRotation().y();
        transform.transform.rotation.z = map_odom.getRotation().z();
        transform.transform.rotation.w = map_odom.getRotation().w();
        tf_broadcaster_->sendTransform(transform);
    } catch (const tf2::TransformException& error) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "Cannot compute %s -> %s: %s", map_frame_.c_str(), odom_frame_.c_str(), error.what());
    }
}

void LocSystem::HandleInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
    const auto& orientation = message->pose.pose.orientation;
    Quatd rotation(orientation.w, orientation.x, orientation.y, orientation.z);
    if (rotation.norm() < 1e-6) {
        RCLCPP_WARN(node_->get_logger(), "Ignoring /initialpose with a zero-length orientation quaternion");
        return;
    }

    rotation.normalize();
    const auto& position = message->pose.pose.position;
    loc_->SetExternalPose(rotation, Vec3d(position.x, position.y, position.z));
    loc_started_ = true;
    RCLCPP_INFO(node_->get_logger(), "Received /initialpose; restarting Lightning-LM localization");
}

}  // namespace lightning