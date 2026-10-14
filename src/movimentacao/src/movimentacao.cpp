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
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "movimentacao/Constants.h"
#include "movimentacao/TitanQuad.h"
#include "movimentacao/XDrive.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MovimentacaoVMX : public rclcpp::Node
{
private:
    float cmd_garra_atual_ = 0.0;

    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
    double vx_ = 0.0, vy_ = 0.0, vth_ = 0.0;

    std::mutex encoder_mutex_;
    double r1_lido_ = 0.0, r2_lido_ = 0.0, r3_lido_ = 0.0, r4_lido_ = 0.0;
    rclcpp::Time last_encoder_msg_time_;

    rclcpp::Time last_compute_time_;
    bool first_reading_ = true;
    bool publish_tf_ = true;

    double fator_linear_ = Constants::DEFAULT_FATOR_LINEAR;
    double fator_angular_ = Constants::DEFAULT_FATOR_ANGULAR;

    XDrive cinematica_;

    std::deque<double> v1_hist_, v2_hist_, v3_hist_, v4_hist_;
    const size_t FILTER_WINDOW_SIZE = 5;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr garra_sub_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr enc1_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr enc2_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr enc3_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr enc4_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mot1_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mot2_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mot3_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mot4_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr garra_pub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::TimerBase::SharedPtr main_control_timer_;

    double apply_filter(std::deque<double> &hist, double new_val)
    {
        hist.push_back(new_val);
        if (hist.size() > FILTER_WINDOW_SIZE)
        {
            hist.pop_front();
        }
        double sum = 0.0;
        for (double v : hist)
        {
            sum += v;
        }
        return sum / hist.size();
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double rpm1, rpm2, rpm3, rpm4;
        cinematica_.Drive(msg->linear.x, msg->linear.y, msg->angular.z, rpm1, rpm2, rpm3, rpm4);

        std_msgs::msg::Float64 m1_msg, m2_msg, m3_msg, m4_msg;
        m1_msg.data = std::clamp(rpm1 / Constants::MAX_RPM, -1.0, 1.0);
        m2_msg.data = std::clamp(rpm2 / Constants::MAX_RPM, -1.0, 1.0);
        m3_msg.data = std::clamp(rpm3 / Constants::MAX_RPM, -1.0, 1.0);
        m4_msg.data = std::clamp(rpm4 / Constants::MAX_RPM, -1.0, 1.0);

        mot1_pub_->publish(m1_msg);
        mot2_pub_->publish(m2_msg);
        mot3_pub_->publish(m3_msg);
        mot4_pub_->publish(m4_msg);
    }

    void garra_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        cmd_garra_atual_ = msg->data;
        std_msgs::msg::Float64 garra_msg;
        garra_msg.data = cmd_garra_atual_;
        garra_pub_->publish(garra_msg);
    }

    void enc1_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(encoder_mutex_);
        double val = msg->data;
        if (val > 327.68) val -= 655.36;
        r1_lido_ = val;
        last_encoder_msg_time_ = this->now();
    }

    void enc2_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(encoder_mutex_);
        double val = msg->data;
        if (val > 327.68) val -= 655.36;
        r2_lido_ = val;
        last_encoder_msg_time_ = this->now();
    }

    void enc3_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(encoder_mutex_);
        double val = msg->data;
        if (val > 327.68) val -= 655.36;
        r3_lido_ = val;
        last_encoder_msg_time_ = this->now();
    }

    void enc4_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(encoder_mutex_);
        double val = msg->data;
        if (val > 327.68) val -= 655.36;
        r4_lido_ = val;
        last_encoder_msg_time_ = this->now();
    }

    void control_loop()
    {
        auto now = this->now();
        double r1, r2, r3, r4;
        rclcpp::Time last_msg_time;

        {
            std::lock_guard<std::mutex> lock(encoder_mutex_);
            r1 = r1_lido_;
            r2 = r2_lido_;
            r3 = r3_lido_;
            r4 = r4_lido_;
            last_msg_time = last_encoder_msg_time_;
        }

        if ((now - last_msg_time).seconds() > 0.2 || first_reading_)
        {
            vx_ = 0.0; vy_ = 0.0; vth_ = 0.0;
            v1_hist_.clear(); v2_hist_.clear(); v3_hist_.clear(); v4_hist_.clear();

            if (first_reading_)
            {
                last_compute_time_ = now;
                first_reading_ = false;
            }
            publish_data(now);
            return;
        }

        double dt = (now - last_compute_time_).seconds();
        last_compute_time_ = now;

        if (dt <= 0.0 || dt > 0.1)
        {
            publish_data(now);
            return;
        }

        if (std::abs(r1) > Constants::MAX_RPM_SAFE ||
            std::abs(r2) > Constants::MAX_RPM_SAFE ||
            std::abs(r3) > Constants::MAX_RPM_SAFE ||
            std::abs(r4) > Constants::MAX_RPM_SAFE)
        {
            publish_data(now);
            return;
        }

        double v1_raw = r1 * (2.0 * M_PI / 60.0);
        double v2_raw = r2 * (2.0 * M_PI / 60.0);
        double v3_raw = r3 * (2.0 * M_PI / 60.0);
        double v4_raw = r4 * (2.0 * M_PI / 60.0);

        double v1 = apply_filter(v1_hist_, v1_raw);
        double v2 = apply_filter(v2_hist_, v2_raw);
        double v3 = apply_filter(v3_hist_, v3_raw);
        double v4 = apply_filter(v4_hist_, v4_raw);

        if (std::abs(v1) < 0.01 && std::abs(v2) < 0.01 && std::abs(v3) < 0.01 && std::abs(v4) < 0.01)
        {
            vx_ = 0.0; vy_ = 0.0; vth_ = 0.0;
        }
        else
        {
            double vx_calc, vy_calc, vth_calc;
            cinematica_.getRobotVelocity(v1, v2, v3, v4, vx_calc, vy_calc, vth_calc);

            vx_ = vx_calc * fator_linear_;
            vy_ = vy_calc * fator_linear_;
            vth_ = vth_calc * fator_angular_;
        }

        double delta_x = (vx_ * std::cos(theta_) - vy_ * std::sin(theta_)) * dt;
        double delta_y = (vx_ * std::sin(theta_) + vy_ * std::cos(theta_)) * dt;
        double delta_th = vth_ * dt;

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_th;

        if (theta_ > M_PI) theta_ -= 2.0 * M_PI;
        if (theta_ < -M_PI) theta_ += 2.0 * M_PI;

        publish_data(now);
    }

    void publish_data(const rclcpp::Time &stamp)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

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
        odom.twist.covariance[0] = 0.01;
        odom.twist.covariance[7] = 0.01;
        odom.twist.covariance[35] = 0.02;

        odom_pub_->publish(odom);

        if (publish_tf_)
        {
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
    }

public:
    MovimentacaoVMX(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("no_movimentacao", options)
    {
        last_encoder_msg_time_ = this->now();
        last_compute_time_ = this->now();

        this->declare_parameter("wheel_radius", Constants::WHEEL_RADIUS);
        this->declare_parameter("robot_radius", Constants::ROBOT_RADIUS);
        this->declare_parameter("cmd_inv1", 1.0);
        this->declare_parameter("cmd_inv2", 1.0);
        this->declare_parameter("cmd_inv3", 1.0);
        this->declare_parameter("cmd_inv4", 1.0);
        this->declare_parameter("enc_inv1", 1.0);
        this->declare_parameter("enc_inv2", 1.0);
        this->declare_parameter("enc_inv3", 1.0);
        this->declare_parameter("enc_inv4", 1.0);
        this->declare_parameter("gain1", 1.0);
        this->declare_parameter("gain2", 1.0);
        this->declare_parameter("gain3", 1.0);
        this->declare_parameter("gain4", 1.0);
        this->declare_parameter("fator_linear", Constants::DEFAULT_FATOR_LINEAR);
        this->declare_parameter("fator_angular", Constants::DEFAULT_FATOR_ANGULAR);
        this->declare_parameter("publish_tf", true);

        double wheel_radius = this->get_parameter("wheel_radius").as_double();
        double robot_radius = this->get_parameter("robot_radius").as_double();
        double cmd_i1 = this->get_parameter("cmd_inv1").as_double();
        double cmd_i2 = this->get_parameter("cmd_inv2").as_double();
        double cmd_i3 = this->get_parameter("cmd_inv3").as_double();
        double cmd_i4 = this->get_parameter("cmd_inv4").as_double();
        double enc_i1 = this->get_parameter("enc_inv1").as_double();
        double enc_i2 = this->get_parameter("enc_inv2").as_double();
        double enc_i3 = this->get_parameter("enc_inv3").as_double();
        double enc_i4 = this->get_parameter("enc_inv4").as_double();
        double g1 = this->get_parameter("gain1").as_double();
        double g2 = this->get_parameter("gain2").as_double();
        double g3 = this->get_parameter("gain3").as_double();
        double g4 = this->get_parameter("gain4").as_double();

        fator_linear_ = this->get_parameter("fator_linear").as_double();
        fator_angular_ = this->get_parameter("fator_angular").as_double();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();

        cinematica_.setRadius(wheel_radius, robot_radius);
        cinematica_.setCmdInversions(cmd_i1, cmd_i2, cmd_i3, cmd_i4);
        cinematica_.setEncInversions(enc_i1, enc_i2, enc_i3, enc_i4);
        cinematica_.setGains(g1, g2, g3, g4);

        mot1_pub_ = this->create_publisher<std_msgs::msg::Float64>(Constants::MOTOR0_CMD_TOPIC, 10);
        mot2_pub_ = this->create_publisher<std_msgs::msg::Float64>(Constants::MOTOR1_CMD_TOPIC, 10);
        mot3_pub_ = this->create_publisher<std_msgs::msg::Float64>(Constants::MOTOR2_CMD_TOPIC, 10);
        mot4_pub_ = this->create_publisher<std_msgs::msg::Float64>(Constants::MOTOR3_CMD_TOPIC, 10);
        garra_pub_ = this->create_publisher<std_msgs::msg::Float64>(Constants::SERVO_GARRA_TOPIC, 10);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MovimentacaoVMX::cmd_vel_callback, this, _1));

        garra_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "cmd_garra", 10, std::bind(&MovimentacaoVMX::garra_callback, this, _1));

        enc1_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            Constants::ENCODER0_RPM_TOPIC, 10, std::bind(&MovimentacaoVMX::enc1_callback, this, _1));
        enc2_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            Constants::ENCODER1_RPM_TOPIC, 10, std::bind(&MovimentacaoVMX::enc2_callback, this, _1));
        enc3_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            Constants::ENCODER2_RPM_TOPIC, 10, std::bind(&MovimentacaoVMX::enc3_callback, this, _1));
        enc4_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            Constants::ENCODER3_RPM_TOPIC, 10, std::bind(&MovimentacaoVMX::enc4_callback, this, _1));

        main_control_timer_ = this->create_wall_timer(
            20ms, std::bind(&MovimentacaoVMX::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "No_movimentacao X-Drive iniciado com sucesso.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovimentacaoVMX>());
    rclcpp::shutdown();
    return 0;
}