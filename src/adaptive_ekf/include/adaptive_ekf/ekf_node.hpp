#ifndef ADAPTIVE_EKF_NODE_HPP
#define ADAPTIVE_EKF_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

/*
 * Adaptive EKF for planar mobile robot.
 * Designed to reduce wheel-odometry influence during slip
 * (e.g. icy / low-traction surfaces).
 */

using namespace Eigen;

class AdaptiveEKF : public rclcpp::Node
{
public:
    explicit AdaptiveEKF();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    void predict(double dt);
    void update_with_odom(double v_meas, double w_meas, bool slipping);
    void update_with_gps(double x, double y);

    void latlon_to_xy(double lat, double lon, double &x, double &y);
    void publish_outputs(const rclcpp::Time &stamp);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_ekf_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    VectorXd x_state_;   
    MatrixXd P_;        

    MatrixXd Q_;         
    MatrixXd R_odom_;    
    MatrixXd R_gps_;     

    double imu_yaw_rate_;       

    bool first_fix_;            
    double origin_lat_;
    double origin_lon_;

rclcpp::Time t_last_;
};

#endif  
