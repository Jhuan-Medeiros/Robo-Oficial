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

#include "movimentacao/Constants.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class KiwiKinematics
{
private:
    double wheel_radius_ = 0.05;
    double robot_radius_ = 0.15;

public:
    void setRadius(double wheel_r, double robot_r)
    {
        wheel_radius_ = wheel_r;
        robot_radius_ = robot_r;
    }

    // Cinemática Inversa (Invertido o eixo X para o robô andar para trás com comando para frente)
    void getMotorSpeeds(double vx, double vy, double vth,
                        double &rpm0, double &rpm1, double &rpm2, double &rpm3)
    {
        const double sqrt3_2 = std::sqrt(3.0) / 2.0;

        // Adicionado o sinal de menos (-) no vx para inverter o sentido de avanço
        double v0 =  sqrt3_2 * (-vx) + 0.5 * vy + robot_radius_ * vth;
        double v1 = -sqrt3_2 * (-vx) + 0.5 * vy + robot_radius_ * vth;
        double v2 =  0.0     * (-vx) - 1.0 * vy + robot_radius_ * vth;

        const double mps_to_rpm = 60.0 / (2.0 * M_PI * wheel_radius_);

        rpm0 = v0 * mps_to_rpm;
        rpm1 = v1 * mps_to_rpm;
        rpm2 = v2 * mps_to_rpm;
        rpm3 = 0.0;
    }

    // Cinemática Direta (Invertido o vx correspondente aos encoders)
    void getRobotVelocity(double v0, double v1, double v2, double v3,
                          double &vx, double &vy, double &vth)
    {
        (void)v3;
        const double sqrt3 = std::sqrt(3.0);

        // Adicionado o sinal de menos (-) para que a odometria acompanhe o novo sentido
        vx  = -((v0 - v1) / sqrt3);
        vy  = (v0 + v1 - 2.0 * v2) / 3.0;
        vth = (v0 + v1 + v2) / (3.0 * robot_radius_);
    }
};

class Movimentacao : public rclcpp::Node
{
private:
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
    double vx_ = 0.0, vy_ = 0.0, vth_ = 0.0;

    double wheel_radius_ = Constants::WHEEL_RADIUS;
    double robot_radius_ = Constants::ROBOT_RADIUS;
    double fator_linear_ = 1.0;
    double fator_angular_ = 1.0;

    KiwiKinematics kinematics_;

    std::mutex encoder_mutex_;
    double enc_pos_[4] = {0.0, 0.0, 0.0, 0.0};
    double enc_prev_[4] = {0.0, 0.0, 0.0, 0.0};
    bool encoder_initialized_ = false;

    rclcpp::Time last_encoder_msg_time_;
    rclcpp::Time last_compute_time_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr garra_sub_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr enc_sub_[4];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mot_pub_[4];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr garra_pub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr main_control_timer_;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double rpm[4];

        kinematics_.getMotorSpeeds(
            msg->linear.x, msg->linear.y, msg->angular.z,
            rpm[0], rpm[1], rpm[2], rpm[3]);

        for (int i = 0; i < 4; ++i)
        {
            double cmd = std::clamp(rpm[i] / Constants::MAX_RPM, -1.0, 1.0);

            std_msgs::msg::Float64 out_msg;
            out_msg.data = cmd;
            mot_pub_[i]->publish(out_msg);
        }
    }

    void garra_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        garra_pub_->publish(*msg);
    }

    void encoder_callback(size_t index, const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(encoder_mutex_);
        enc_pos_[index] = msg->data;
        last_encoder_msg_time_ = this->now();
    }

    void publish_data(const rclcpp::Time &stamp)
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

        odom.pose.covariance[0] = 0.01;
        odom.pose.covariance[7] = 0.01;
        odom.pose.covariance[35] = 0.02;

        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
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
        double p[4];
        rclcpp::Time last_enc_time;

        {
            std::lock_guard<std::mutex> lock(encoder_mutex_);
            for (int i = 0; i < 4; ++i)
                p[i] = enc_pos_[i];
            last_enc_time = last_encoder_msg_time_;
        }

        if (!encoder_initialized_)
        {
            for (int i = 0; i < 4; ++i)
                enc_prev_[i] = p[i];
            encoder_initialized_ = true;
            last_compute_time_ = now;
            publish_data(now);
            return;
        }

        double dt = (now - last_compute_time_).seconds();
        last_compute_time_ = now;

        if (dt <= 0.0 || dt > 0.20)
        {
            for (int i = 0; i < 4; ++i)
                enc_prev_[i] = p[i];
            publish_data(now);
            return;
        }

        double v_wheel[4];
        for (int i = 0; i < 4; ++i)
        {
            double d_meters = p[i] - enc_prev_[i];
            enc_prev_[i] = p[i];
            v_wheel[i] = d_meters / dt;
        }

        kinematics_.getRobotVelocity(v_wheel[0], v_wheel[1], v_wheel[2], v_wheel[3], vx_, vy_, vth_);

        vx_ *= fator_linear_;
        vy_ *= fator_linear_;
        vth_ *= fator_angular_;

        x_ += (vx_ * std::cos(theta_) - vy_ * std::sin(theta_)) * dt;
        y_ += (vx_ * std::sin(theta_) + vy_ * std::cos(theta_)) * dt;
        theta_ += vth_ * dt;

        while (theta_ > M_PI)
            theta_ -= 2.0 * M_PI;
        while (theta_ < -M_PI)
            theta_ += 2.0 * M_PI;

        publish_data(now);
    }

public:
    Movimentacao(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
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
        kinematics_.setRadius(wheel_radius_, robot_radius_);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&Movimentacao::cmd_vel_callback, this, _1));

        garra_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "cmd_garra", 10, std::bind(&Movimentacao::garra_callback, this, _1));
        garra_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            Constants::SERVO_GARRA_TOPIC, 10);

        mot_pub_[0] = this->create_publisher<std_msgs::msg::Float64>(Constants::MOTOR0_CMD_TOPIC, 10);
        mot_pub_[1] = this->create_publisher<std_msgs::msg::Float64>(Constants::MOTOR1_CMD_TOPIC, 10);
        mot_pub_[2] = this->create_publisher<std_msgs::msg::Float64>(Constants::MOTOR2_CMD_TOPIC, 10);
        mot_pub_[3] = this->create_publisher<std_msgs::msg::Float64>(Constants::MOTOR3_CMD_TOPIC, 10);

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

        last_encoder_msg_time_ = this->now();
        last_compute_time_ = this->now();

        main_control_timer_ = this->create_wall_timer(
            20ms, std::bind(&Movimentacao::control_loop, this));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Movimentacao>());
    rclcpp::shutdown();
    return 0;
}