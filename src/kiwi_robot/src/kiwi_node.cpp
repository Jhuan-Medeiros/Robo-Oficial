#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>

#include "kiwi_robot/Constants.h"
#include "kiwi_robot/KiwiKinematics.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class KiwiNode : public rclcpp::Node
{
private:
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
    double vx_ = 0.0, vy_ = 0.0, vth_ = 0.0;

    double cmd_vx_ = 0.0, cmd_vy_ = 0.0, cmd_vth_ = 0.0;

    double wheel_radius_ = Constants::WHEEL_RADIUS;
    double robot_radius_ = Constants::ROBOT_RADIUS;
    double fator_linear_ = 1.0;
    double fator_angular_ = 1.0;

    KiwiKinematics kinematics_;

    std::mutex data_mutex_;
    double enc_pos_[4] = {0.0, 0.0, 0.0, 0.0};
    double enc_prev_[4] = {0.0, 0.0, 0.0, 0.0};
    bool enc_init_[4]  = {false, false, false, false};

    rclcpp::Time last_compute_time_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr enc_sub_[4];

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mot_pub_[3];
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            cmd_vx_  = msg->linear.x;
            cmd_vy_  = msg->linear.y;
            cmd_vth_ = msg->angular.z;
        }

        double rpm[4];
        kinematics_.inverseKinematics(
            msg->linear.x, msg->linear.y, msg->angular.z,
            rpm[0], rpm[1], rpm[2], rpm[3]);

        // Publica apenas nos motores 0, 1 e 2 (m_3 fica livre para o elevador do OMS)
        for (int i = 0; i < 3; ++i)
        {
            double duty = std::clamp(rpm[i] / Constants::MAX_RPM, -1.0, 1.0);
            std_msgs::msg::Float64 cmd_msg;
            cmd_msg.data = duty;
            mot_pub_[i]->publish(cmd_msg);
        }
    }

    void encoder_callback(size_t index, const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        enc_pos_[index] = msg->data;
        if (!enc_init_[index])
        {
            enc_prev_[index] = msg->data;
            enc_init_[index] = true;
        }
    }

    void publish_odometry(const rclcpp::Time &stamp)
    {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = vx_;
        odom.twist.twist.linear.y = vy_;
        odom.twist.twist.angular.z = vth_;

        odom.pose.covariance[0]  = 0.005;
        odom.pose.covariance[7]  = 0.005;
        odom.pose.covariance[35] = 0.01;

        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp + rclcpp::Duration::from_seconds(0.03);
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }

    void control_loop()
    {
        const rclcpp::Time now = this->now();
        double dt = (now - last_compute_time_).seconds();
        last_compute_time_ = now;

        if (dt <= 0.0 || dt > 0.5)
        {
            publish_odometry(now);
            return;
        }

        double d[3] = {0.0, 0.0, 0.0};
        double c_vx, c_vy, c_vth;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (int i = 0; i < 3; ++i)
            {
                if (enc_init_[i])
                {
                    d[i] = enc_pos_[i] - enc_prev_[i];
                    enc_prev_[i] = enc_pos_[i];
                }
            }
            c_vx  = cmd_vx_;
            c_vy  = cmd_vy_;
            c_vth = cmd_vth_;
        }

        double dx_body = 0.0, dy_body = 0.0, dth = 0.0;

        if (std::abs(c_vx) > 0.001)
        {
            double sign_x = (c_vx > 0.0) ? 1.0 : -1.0;
            double mean_d = (std::abs(d[0]) + std::abs(d[1])) / 2.0;
            dx_body = sign_x * mean_d;
        }

        if (std::abs(c_vy) > 0.001)
        {
            double sign_y = (c_vy > 0.0) ? 1.0 : -1.0;
            dy_body = sign_y * ((std::abs(d[0]) + std::abs(d[1]) + 2.0 * std::abs(d[2])) / 4.0);
        }

        if (std::abs(c_vth) > 0.001)
        {
            double sign_th = (c_vth > 0.0) ? 1.0 : -1.0;
            dth = sign_th * ((std::abs(d[0]) + std::abs(d[1]) + std::abs(d[2])) / (3.0 * robot_radius_));
        }

        dx_body *= fator_linear_;
        dy_body *= fator_linear_;
        dth     *= fator_angular_;

        vx_  = dx_body / dt;
        vy_  = dy_body / dt;
        vth_ = dth / dt;

        if (std::abs(vx_) < 0.002) { vx_ = 0.0; dx_body = 0.0; }
        if (std::abs(vy_) < 0.002) { vy_ = 0.0; dy_body = 0.0; }
        if (std::abs(vth_) < 0.005) { vth_ = 0.0; dth = 0.0; }

        double theta_mid = theta_ + 0.5 * dth;
        x_     += dx_body * std::cos(theta_mid) - dy_body * std::sin(theta_mid);
        y_     += dx_body * std::sin(theta_mid) + dy_body * std::cos(theta_mid);
        theta_ += dth;

        while (theta_ > M_PI)  theta_ -= 2.0 * M_PI;
        while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

        publish_odometry(now);
    }

public:
    KiwiNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("no_movimentacao", options)
    {
        this->declare_parameter("wheel_radius", Constants::WHEEL_RADIUS);
        this->declare_parameter("robot_radius", Constants::ROBOT_RADIUS);
        this->declare_parameter("fator_linear", 1.0);
        this->declare_parameter("fator_angular", 1.0);

        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        robot_radius_ = this->get_parameter("robot_radius").as_double();
        fator_linear_ = this->get_parameter("fator_linear").as_double();
        fator_angular_ = this->get_parameter("fator_angular").as_double();

        kinematics_.setGeometry(wheel_radius_, robot_radius_);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&KiwiNode::cmd_vel_callback, this, _1));

        mot_pub_[0] = this->create_publisher<std_msgs::msg::Float64>(Constants::MOTOR0_CMD_TOPIC, 10);
        mot_pub_[1] = this->create_publisher<std_msgs::msg::Float64>(Constants::MOTOR1_CMD_TOPIC, 10);
        mot_pub_[2] = this->create_publisher<std_msgs::msg::Float64>(Constants::MOTOR2_CMD_TOPIC, 10);

        const char *enc_topics[4] = {
            Constants::ENCODER0_TOPIC, Constants::ENCODER1_TOPIC,
            Constants::ENCODER2_TOPIC, Constants::ENCODER3_TOPIC};

        for (size_t i = 0; i < 4; ++i)
        {
            enc_sub_[i] = this->create_subscription<std_msgs::msg::Float64>(
                enc_topics[i], 10,
                [this, i](const std_msgs::msg::Float64::SharedPtr msg)
                {
                    this->encoder_callback(i, msg);
                });
        }

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        last_compute_time_ = this->now();
        publish_odometry(last_compute_time_);

        control_timer_ = this->create_wall_timer(
            20ms, std::bind(&KiwiNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), ">>> KiwiNode Iniciado (m_0, m_1, m_2 ativos | m_3 livre para OMS) <<<");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KiwiNode>());
    rclcpp::shutdown();
    return 0;
}
