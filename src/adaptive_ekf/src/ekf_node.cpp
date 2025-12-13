#include "adaptive_ekf/ekf_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

/*
 * State: [x, y, yaw, v, w]
 * Wheels -> velocity update
 * IMU    -> slip detection
 * GPS    -> position correction 
*/


AdaptiveEKF::AdaptiveEKF() : Node("adaptive_ekf_node")
{
    // -------------------- initial staes fixing --------------------
    x_state_.setZero(5);

    P_.setIdentity(5, 5);
    P_ *= 0.1; 
    Q_.setZero(5, 5);
    Q_(0, 0) = 0.01;  
    Q_(1, 1) = 0.01;  
    Q_(2, 2) = 0.01;  
    Q_(3, 3) = 0.5;
    Q_(4, 4) = 0.5;   

    // -------------------- measurement noise --------------------
    R_odom_.setIdentity(2, 2);
    R_odom_ *= 0.1;   

    R_gps_.setIdentity(2, 2);
    R_gps_ *= 2.0;    

    imu_yaw_rate_ = 0.0;
    first_fix_ = false;

    // -------------------- ROS interfaces --------------------
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos,
        std::bind(&AdaptiveEKF::odom_callback, this, std::placeholders::_1));

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", qos,
        std::bind(&AdaptiveEKF::imu_callback, this, std::placeholders::_1));

    sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", 10,
        std::bind(&AdaptiveEKF::gps_callback, this, std::placeholders::_1));

    pub_ekf_ = create_publisher<nav_msgs::msg::Odometry>("/ekf_odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Adaptive EKF initialized (5-state planar model)");
}

// ========================== imu ==========================
void AdaptiveEKF::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_yaw_rate_ = msg->angular_velocity.z;
}

// ========================== odom ==========================
void AdaptiveEKF::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    const rclcpp::Time t = msg->header.stamp;

    if (t_last_.nanoseconds() == 0) {
        t_last_ = t;
        return;
    }

    const double dt = (t - t_last_).seconds();
    t_last_ = t;

    // -------------------- prediction --------------------
    predict(dt);

    const double v_meas = msg->twist.twist.linear.x;
    const double w_meas = msg->twist.twist.angular.z;

    // -------------------- slipdetection --------------------
    const double yaw_error = std::abs(w_meas - imu_yaw_rate_);
    const bool slipping = yaw_error > 0.3;  // tuned empirically

    // -------------------- update --------------------
    update_with_odom(v_meas, w_meas, slipping);

    publish_outputs(t);
}

// ========================== predict ==========================
void AdaptiveEKF::predict(double dt)
{
    const double theta = x_state_(2);
    const double v = x_state_(3);
    const double w = x_state_(4);

    x_state_(0) += v * std::cos(theta) * dt;
    x_state_(1) += v * std::sin(theta) * dt;
    x_state_(2) += w * dt;

    MatrixXd F = MatrixXd::Identity(5, 5);
    F(0, 2) = -v * std::sin(theta) * dt;
    F(1, 2) =  v * std::cos(theta) * dt;
    F(0, 3) =  std::cos(theta) * dt;
    F(1, 3) =  std::sin(theta) * dt;
    F(2, 4) =  dt;

    P_ = F * P_ * F.transpose() + Q_;
}

// ========================== odom_U ==========================
void AdaptiveEKF::update_with_odom(double v, double w, bool slipping)
{
    VectorXd z(2);
    z << v, w;

    MatrixXd H = MatrixXd::Zero(2, 5);
    H(0, 3) = 1.0;
    H(1, 4) = 1.0;

    MatrixXd R = R_odom_;
    if (slipping) {
        R *= 1000.0;  
    }

    const MatrixXd S = H * P_ * H.transpose() + R;
    const MatrixXd K = P_ * H.transpose() * S.inverse();

    const VectorXd innovation = z - H * x_state_;
    x_state_ += K * innovation;


    const MatrixXd I = MatrixXd::Identity(5, 5);
    P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();
}

// ========================== gps ==========================
void AdaptiveEKF::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{

    if (std::abs(msg->latitude) < 1e-6 && std::abs(msg->longitude) < 1e-6) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "GPS is reporting 0.0! Ignoring this message.");
        return;
    }
    
    if (!first_fix_) {
        origin_lat_ = msg->latitude;
        origin_lon_ = msg->longitude;
        first_fix_ = true;
        return;
    }

    double x, y;
    latlon_to_xy(msg->latitude, msg->longitude, x, y);
    update_with_gps(x, y);
}

// ========================== gps_U ==========================

void AdaptiveEKF::update_with_gps(double x, double y)
{
    VectorXd z(2);
    z << x, y;

    MatrixXd H = MatrixXd::Zero(2, 5);
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    const MatrixXd S = H * P_ * H.transpose() + R_gps_;
    const MatrixXd K = P_ * H.transpose() * S.inverse();

    const VectorXd innovation = z - H * x_state_;
    x_state_ += K * innovation;

    const MatrixXd I = MatrixXd::Identity(5, 5);
    P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R_gps_ * K.transpose();
}

// ========================== UTILS ==========================
void AdaptiveEKF::latlon_to_xy(double lat, double lon, double &x, double &y)
{
    constexpr double R = 6378137.0; 

    const double lat_r = lat * M_PI / 180.0;
    const double lon_r = lon * M_PI / 180.0;
    const double lat0 = origin_lat_ * M_PI / 180.0;
    const double lon0 = origin_lon_ * M_PI / 180.0;

    x = R * (lon_r - lon0) * std::cos(lat0);
    y = R * (lat_r - lat0);
}

void AdaptiveEKF::publish_outputs(const rclcpp::Time &stamp)
{
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_state_(0);
    odom.pose.pose.position.y = x_state_(1);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, x_state_(2));
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom.twist.twist.linear.x = x_state_(3);
    odom.twist.twist.angular.z = x_state_(4);

    pub_ekf_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = odom.child_frame_id;
    tf.transform.translation.x = x_state_(0);
    tf.transform.translation.y = x_state_(1);
    tf.transform.rotation = odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdaptiveEKF>());
    rclcpp::shutdown();
    return 0;
}
