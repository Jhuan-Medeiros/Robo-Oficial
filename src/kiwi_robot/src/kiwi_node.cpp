#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "kiwi_robot/Constants.h"
#include "kiwi_robot/KiwiKinematics.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class KiwiNode : public rclcpp::Node
{
private:
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
    double vx_ = 0.0, vy_ = 0.0, vth_ = 0.0;

    double wheel_radius_ = 0.0500;
    double robot_radius_ = 0.2000;
    bool publish_tf_     = true;

    KiwiKinematics kinematics_;

    double enc_pos_[4]  = {0.0, 0.0, 0.0, 0.0};
    double enc_prev_[4] = {0.0, 0.0, 0.0, 0.0};
    bool enc_init_[4]   = {false, false, false, false};

    bool is_moving_                      = false;
    bool script_rodando_                 = false;
    bool piscando_laranja_               = false;
    bool blink_state_                    = false;
    int  blink_tick_count_               = 0;

    bool emerg_pressionado_              = false;
    bool aguardando_retomada_pos_emerg_  = false;
    bool stop_pressionado_               = false;
    bool stop_acionado_                  = false;

    bool start_init_                     = false;
    bool start_repouso_                  = false;
    bool start_apertado_                 = false;
    bool start_ant_                      = false;

    bool reset_init_                     = false;
    bool reset_repouso_                  = false;
    bool reset_apertado_                 = false;
    bool reset_ant_                      = false;

    int  led_state_atual_                = -1;

    rclcpp::Time last_compute_time_;
    rclcpp::Time last_cmd_time_;
    rclcpp::Time last_script_msg_time_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr enc_sub_[4];
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr script_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr laranja_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emerg_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mot_pub_[3];
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_stopped_pub_; // Pino 18 (Vermelho)
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_running_pub_; // Pino 19 (Verde)
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_stop_pub_;    // Pino 15 (LED Stop)
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_reset_pub_;   // Pino 16 (LED Reset / Laranja)
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_start_pub_;   // Pino 17 (LED Start)

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr painel_start_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr painel_reset_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr painel_stop_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr painel_emerg_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    void publicar_botoes_painel()
    {
        std_msgs::msg::Bool m_start, m_reset, m_stop, m_emerg;
        m_start.data = start_apertado_;
        m_reset.data = reset_apertado_;
        m_stop.data  = stop_pressionado_;
        m_emerg.data = emerg_pressionado_;

        painel_start_pub_->publish(m_start);
        painel_reset_pub_->publish(m_reset);
        painel_stop_pub_->publish(m_stop);
        painel_emerg_pub_->publish(m_emerg);
    }

    void set_leds(bool vermelho, bool verde, bool laranja_reset, bool led_start, bool led_stop)
    {
        std_msgs::msg::Bool msg_verm, msg_verd, msg_lar, msg_st_in, msg_stop;
        msg_verm.data  = vermelho;
        msg_verd.data  = verde;
        msg_lar.data   = laranja_reset;
        msg_st_in.data = led_start;
        msg_stop.data  = led_stop;

        led_stopped_pub_->publish(msg_verm);
        led_running_pub_->publish(msg_verd);
        led_reset_pub_->publish(msg_lar);
        led_start_pub_->publish(msg_st_in);
        led_stop_pub_->publish(msg_stop);
    }

    void zerar_odometria_inicio()
    {
        x_     = 0.0;
        y_     = 0.0;
        theta_ = 0.0;
        vx_    = 0.0;
        vy_    = 0.0;
        vth_   = 0.0;
    }

    void update_leds(const rclcpp::Time &now)
    {
        // Só desativa script_rodando_ se não estiver piscando laranja e passar mais de 3.0s sem sinal
        if (script_rodando_ && !piscando_laranja_ && (now - last_script_msg_time_).seconds() > 3.0)
        {
            script_rodando_ = false;
        }

        if (emerg_pressionado_)
        {
            if (led_state_atual_ != 4)
            {
                led_state_atual_ = 4;
                set_leds(true, false, false, false, true);
            }
            return;
        }

        if (aguardando_retomada_pos_emerg_ || stop_acionado_)
        {
            if (led_state_atual_ != 3)
            {
                led_state_atual_ = 3;
                set_leds(true, false, false, true, true);
            }
            return;
        }

        // Durante a validação de 5s no ponto: LED Stopped (18) SEMPRE APAGADO, Running (19) ACESO e Reset (16) PISCANDO
        if (piscando_laranja_)
        {
            led_state_atual_ = 2;
            blink_tick_count_++;
            if (blink_tick_count_ >= 10)
            {
                blink_tick_count_ = 0;
                blink_state_ = !blink_state_;
                set_leds(false, true, blink_state_, false, false);
            }
            return;
        }

        int novo_estado = script_rodando_ ? 1 : 0;

        if (novo_estado != led_state_atual_)
        {
            led_state_atual_ = novo_estado;
            if (script_rodando_)
            {
                set_leds(false, true, false, false, false);
            }
            else
            {
                set_leds(true, false, reset_apertado_, true, false);
            }
        }
    }

    void force_stop_motors()
    {
        for (int i = 0; i < 3; ++i)
        {
            std_msgs::msg::Float64 cmd_msg;
            cmd_msg.data = 0.0;
            mot_pub_[i]->publish(cmd_msg);
        }
        is_moving_ = false;
    }

    void send_motor_cmd(double d0, double d1, double d2)
    {
        if (emerg_pressionado_ || aguardando_retomada_pos_emerg_ || stop_acionado_)
        {
            force_stop_motors();
            return;
        }

        if (!enc_init_[0] || !enc_init_[2] || !enc_init_[3]) return;

        double duty[3] = {d0, d1, d2};
        for (int i = 0; i < 3; ++i)
        {
            std_msgs::msg::Float64 cmd_msg;
            cmd_msg.data = duty[i];
            mot_pub_[i]->publish(cmd_msg);
        }
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (emerg_pressionado_ || aguardando_retomada_pos_emerg_ || stop_acionado_)
        {
            force_stop_motors();
            return;
        }

        double rpm[4];
        kinematics_.inverseKinematics(
            msg->linear.x, msg->linear.y, msg->angular.z,
            rpm[0], rpm[1], rpm[2], rpm[3]);

        // Piso de 11% de PWM: vence o atrito nas curvas sem girar rápido demais
        constexpr double MIN_DUTY = 0.11;
        double d[3];
        for (int i = 0; i < 3; ++i)
        {
            double raw = rpm[i] / Constants::MAX_RPM;
            if (std::abs(raw) < 0.002)
            {
                d[i] = 0.0;
            }
            else
            {
                double sign = (raw > 0.0) ? 1.0 : -1.0;
                double mag  = MIN_DUTY + (1.0 - MIN_DUTY) * std::min(std::abs(raw), 1.0);
                d[i] = std::clamp(sign * mag, -1.0, 1.0);
            }
        }

        send_motor_cmd(d[0], d[1], d[2]);

        is_moving_     = (d[0] != 0.0 || d[1] != 0.0 || d[2] != 0.0);
        last_cmd_time_ = this->now();
    }

    void encoder_callback(size_t index, const std_msgs::msg::Float64::SharedPtr msg)
    {
        enc_pos_[index] = msg->data;
        if (!enc_init_[index])
        {
            enc_prev_[index] = msg->data;
            enc_init_[index] = true;
            RCLCPP_INFO(this->get_logger(), ">>> Encoder %zu conectado! <<<", index);
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

        odom.twist.twist.linear.x  = vx_;
        odom.twist.twist.linear.y  = vy_;
        odom.twist.twist.angular.z = vth_;

        odom.pose.covariance[0]  = 0.01;
        odom.pose.covariance[7]  = 0.01;
        odom.pose.covariance[35] = 0.05;

        odom.twist.covariance[0]  = 0.01;
        odom.twist.covariance[7]  = 0.01;
        odom.twist.covariance[35] = 0.05;

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

    void control_loop()
    {
        const rclcpp::Time now = this->now();
        double dt = (now - last_compute_time_).seconds();
        last_compute_time_ = now;

        if (emerg_pressionado_ || aguardando_retomada_pos_emerg_ || stop_acionado_)
        {
            force_stop_motors();
        }
        else if (is_moving_ && (now - last_cmd_time_).seconds() > 0.5)
        {
            force_stop_motors();
        }

        publicar_botoes_painel();
        update_leds(now);

        if (dt <= 0.0 || dt > 0.5)
        {
            publish_odometry(now);
            return;
        }

        double raw_d[4] = {0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < 4; ++i)
        {
            if (enc_init_[i])
            {
                raw_d[i] = enc_pos_[i] - enc_prev_[i];
                enc_prev_[i] = enc_pos_[i];
            }
        }

        if (emerg_pressionado_ || aguardando_retomada_pos_emerg_)
        {
            vx_  = 0.0;
            vy_  = 0.0;
            vth_ = 0.0;
            publish_odometry(now);
            return;
        }

        double dx_body = 0.0, dy_body = 0.0, dth = 0.0;
        kinematics_.forwardKinematics(raw_d[0], raw_d[2], raw_d[3], dx_body, dy_body, dth);

        vx_  = dx_body / dt;
        vy_  = dy_body / dt;
        vth_ = dth / dt;

        if (std::abs(vx_) < 0.0005) vx_ = 0.0;
        if (std::abs(vy_) < 0.0005) vy_ = 0.0;
        if (std::abs(vth_) < 0.001) vth_ = 0.0;

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
        this->declare_parameter("wheel_radius", 0.0500);
        this->declare_parameter("robot_radius", 0.2000);
        this->declare_parameter("publish_tf", true);

        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        robot_radius_ = this->get_parameter("robot_radius").as_double();
        publish_tf_   = this->get_parameter("publish_tf").as_bool();

        kinematics_.setGeometry(wheel_radius_, robot_radius_);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 1, std::bind(&KiwiNode::cmd_vel_callback, this, _1));

        mot_pub_[0] = this->create_publisher<std_msgs::msg::Float64>(Constants::MOTOR0_CMD_TOPIC, 1);
        mot_pub_[1] = this->create_publisher<std_msgs::msg::Float64>(Constants::MOTOR2_CMD_TOPIC, 1);
        mot_pub_[2] = this->create_publisher<std_msgs::msg::Float64>(Constants::MOTOR3_CMD_TOPIC, 1);

        led_stopped_pub_ = this->create_publisher<std_msgs::msg::Bool>("/led_stopped/cmd", 10);
        led_running_pub_ = this->create_publisher<std_msgs::msg::Bool>("/led_running/cmd", 10);
        led_stop_pub_    = this->create_publisher<std_msgs::msg::Bool>("/led_stop/cmd", 10);
        led_reset_pub_   = this->create_publisher<std_msgs::msg::Bool>("/led_reset/cmd", 10);
        led_start_pub_   = this->create_publisher<std_msgs::msg::Bool>("/led_start/cmd", 10);

        painel_start_pub_ = this->create_publisher<std_msgs::msg::Bool>("/painel/start_pressed", 10);
        painel_reset_pub_ = this->create_publisher<std_msgs::msg::Bool>("/painel/reset_pressed", 10);
        painel_stop_pub_  = this->create_publisher<std_msgs::msg::Bool>("/painel/stop_pressed", 10);
        painel_emerg_pub_ = this->create_publisher<std_msgs::msg::Bool>("/painel/emerg_pressed", 10);

        script_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/painel/script_rodando", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                last_script_msg_time_ = this->now();
                bool rodando_novo = msg->data;

                if (rodando_novo && !emerg_pressionado_ && !stop_pressionado_)
                {
                    if (aguardando_retomada_pos_emerg_ || stop_acionado_)
                    {
                        if (aguardando_retomada_pos_emerg_)
                        {
                            zerar_odometria_inicio();
                        }
                        aguardando_retomada_pos_emerg_ = false;
                        stop_acionado_                 = false;
                        led_state_atual_               = -1;
                    }
                }
                if (script_rodando_ != rodando_novo)
                {
                    led_state_atual_ = -1;
                }
                script_rodando_ = rodando_novo;
            });

        emerg_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/emergencia/state", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                bool pressionado_agora = msg->data;
                if (pressionado_agora && !emerg_pressionado_)
                {
                    RCLCPP_ERROR(this->get_logger(), ">>> EMERGÊNCIA ACIONADA (Pino 11)! Motores parados e LED Vermelho aceso! <<<");
                    aguardando_retomada_pos_emerg_ = true;
                    script_rodando_                = false;
                    led_state_atual_               = -1;
                    force_stop_motors();
                }
                else if (!pressionado_agora && emerg_pressionado_)
                {
                    RCLCPP_INFO(this->get_logger(), ">>> Emergência desacionada! Odometria pronta em (0,0,0). Pressione START (ou RESET opcional). <<<");
                    zerar_odometria_inicio();
                    led_state_atual_ = -1;
                }
                emerg_pressionado_ = pressionado_agora;
                publicar_botoes_painel();
            });

        stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/stop/state", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                stop_pressionado_ = msg->data;
                if (stop_pressionado_ && !stop_acionado_)
                {
                    RCLCPP_WARN(this->get_logger(), ">>> STOP ACIONADO (Pino 10)! Robô pausado! <<<");
                    stop_acionado_   = true;
                    script_rodando_  = false;
                    led_state_atual_ = -1;
                    force_stop_motors();
                }
                publicar_botoes_painel();
            });

        start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/start/state", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                if (!start_init_)
                {
                    start_repouso_ = msg->data;
                    start_init_    = true;
                    return;
                }
                start_apertado_ = (msg->data != start_repouso_);
                if (start_apertado_ && !start_ant_)
                {
                    RCLCPP_INFO(this->get_logger(), ">>> BOTÃO START PRESSIONADO (Pino 8)! <<<");
                    if (!emerg_pressionado_ && !stop_pressionado_)
                    {
                        if (aguardando_retomada_pos_emerg_)
                        {
                            zerar_odometria_inicio();
                            aguardando_retomada_pos_emerg_ = false;
                        }
                        if (stop_acionado_)
                        {
                            stop_acionado_ = false;
                        }
                        led_state_atual_ = -1;
                    }
                }
                start_ant_ = start_apertado_;
                publicar_botoes_painel();
            });

        reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/reset/state", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                if (!reset_init_)
                {
                    reset_repouso_ = msg->data;
                    reset_init_    = true;
                    return;
                }

                reset_apertado_ = (msg->data != reset_repouso_);

                if (reset_apertado_ != reset_ant_ && !piscando_laranja_)
                {
                    std_msgs::msg::Bool led_rst;
                    led_rst.data = reset_apertado_;
                    led_reset_pub_->publish(led_rst);
                }

                if (reset_apertado_ && !reset_ant_)
                {
                    if (emerg_pressionado_)
                    {
                        RCLCPP_ERROR(this->get_logger(), ">>> Desacione o botão de EMERGÊNCIA (Pino 11) antes de apertar RESET! <<<");
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), ">>> RESET ACIONADO (Pino 9)! Odometria zerada em (0,0,0). <<<");
                        aguardando_retomada_pos_emerg_ = false;
                        stop_acionado_                 = false;
                        script_rodando_                = false;
                        led_state_atual_               = -1;
                        zerar_odometria_inicio();
                    }
                }
                reset_ant_ = reset_apertado_;
                publicar_botoes_painel();
            });

        laranja_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/painel/piscar_laranja", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                last_script_msg_time_ = this->now();
                piscando_laranja_ = msg->data;
                blink_tick_count_ = 10;
                if (!piscando_laranja_)
                {
                    led_state_atual_ = -1;
                    set_leds(!script_rodando_, script_rodando_, false, !script_rodando_, false);
                }
            });

        const char *enc_topics[4] = {
            Constants::ENCODER0_TOPIC, Constants::ENCODER1_TOPIC,
            Constants::ENCODER2_TOPIC, Constants::ENCODER3_TOPIC};

        for (size_t i = 0; i < 4; ++i)
        {
            enc_sub_[i] = this->create_subscription<std_msgs::msg::Float64>(
                enc_topics[i], 1,
                [this, i](const std_msgs::msg::Float64::SharedPtr msg)
                {
                    this->encoder_callback(i, msg);
                });
        }

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 1);

        last_compute_time_    = this->now();
        last_cmd_time_        = this->now();
        last_script_msg_time_ = this->now();
        publish_odometry(last_compute_time_);

        control_timer_ = this->create_wall_timer(
            50ms, std::bind(&KiwiNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), ">>> KiwiNode V39 (LEDs Estaveis + Torque de Curva) Pronto <<<");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KiwiNode>());
    rclcpp::shutdown();
    return 0;
}