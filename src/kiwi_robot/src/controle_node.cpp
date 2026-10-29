#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <chrono>

using namespace std::chrono_literals;

class ControleNode : public rclcpp::Node {
public:
    ControleNode() : Node("controle_node") {
        pub_vel_  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_elev_ = create_publisher<std_msgs::msg::Float64>("/titan/m_3/cmd", 10);
        pub_s0_   = create_publisher<std_msgs::msg::Float64>("/servo0/cmd", 10);
        pub_s1_   = create_publisher<std_msgs::msg::Float64>("/servo1/cmd", 10);
        pub_s2_   = create_publisher<std_msgs::msg::Float64>("/servo2/cmd", 10);
        sub_joy_  = create_subscription<sensor_msgs::msg::Joy>("/joy", 10,
            [this](const sensor_msgs::msg::Joy::SharedPtr m) {
                std::lock_guard<std::mutex> l(mtx_);
                joy_ = m;
            });
        timer_ = create_wall_timer(20ms, std::bind(&ControleNode::loop, this));
        RCLCPP_INFO(get_logger(), ">>> CONTROLE AJUSTADO (Proporcional + Memoria de Posicao) <<<");
    }

private:
    double dz(double v) { return (std::abs(v) < 0.12) ? 0.0 : v; }

    double lerGatilho(const std::vector<float> &a) {
        double lt = 0.0, rt = 0.0;
        if (a.size() > 2) {
            if (std::abs(a[2]) > 0.05f) lt_ok_ = true;
            if (lt_ok_) lt = std::clamp((1.0 - (double)a[2]) / 2.0, 0.0, 1.0);
        }
        if (a.size() > 5) {
            if (std::abs(a[5]) > 0.05f) rt_ok_ = true;
            if (rt_ok_) rt = std::clamp((1.0 - (double)a[5]) / 2.0, 0.0, 1.0);
        }
        double g = std::max(lt, rt);
        return (g < 0.05) ? 0.0 : g;
    }

    void pubF64(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr p, double v) {
        std_msgs::msg::Float64 m; m.data = v; p->publish(m);
    }

    void loop() {
        sensor_msgs::msg::Joy::SharedPtr j;
        { std::lock_guard<std::mutex> l(mtx_); j = joy_; }
        if (!j || j->axes.size() < 6 || j->buttons.size() < 6) return;

        double vl = (j->buttons[5] == 1) ? 0.63 : 0.35;
        double va = (j->buttons[5] == 1) ? 2.16 : 1.20;

        geometry_msgs::msg::Twist tw;
        tw.linear.x  = dz(j->axes[1]) * vl;
        tw.linear.y  = dz(j->axes[0]) * vl;
        tw.angular.z = dz(j->axes[3]) * va;
        pub_vel_->publish(tw);

        double g = lerGatilho(j->axes);

        // 1. ELEVADOR INVERTIDO (Setinha Cima agora sobe e Baixo desce)
        double v_elev = 0.0;
        if (j->axes.size() > 7 && g > 0.0) {
            if (j->axes[7] > 0.5f) v_elev = -0.65 * g;      // Invertido para subir certo
            else if (j->axes[7] < -0.5f) v_elev = 0.50 * g; // Invertido para descer certo
        }
        pubF64(pub_elev_, v_elev);

        // 2. SERVO 0 360° (Mantém a posição atual ao soltar a setinha)
        if (g > 0.0) {
            if (j->axes.size() > 6) {
                if (j->axes[6] < -0.5f) {
                    val_s0_ = std::clamp(val_s0_ + (1.5 * g), 40.0, 140.0); // Avança
                } else if (j->axes[6] > 0.5f) {
                    val_s0_ = std::clamp(val_s0_ - (1.5 * g), 40.0, 140.0); // Recua
                }
            }
        }
        pubF64(pub_s0_, val_s0_);

        // 3. SERVO 1 (Triangulo / Y = Sobe | X / A = Desce - Usa curso completo 0 a 180)
        if (g > 0.0) {
            if (j->buttons[3] == 1 && j->buttons[0] == 0) { // Triangulo/Y
                ang_s1_ = std::clamp(ang_s1_ + (3.0 * g), 0.0, 180.0);
            } else if (j->buttons[0] == 1 && j->buttons[3] == 0) { // X/A
                ang_s1_ = std::clamp(ang_s1_ - (3.0 * g), 0.0, 180.0);
            }
        }
        pubF64(pub_s1_, ang_s1_);

        // 4. SERVO 2 (Bolinha / B = Fecha | Quadrado / X = Abre - Usa curso completo 0 a 180)
        if (g > 0.0) {
            if (j->buttons[1] == 1 && j->buttons[2] == 0) { // Bola/B
                ang_s2_ = std::clamp(ang_s2_ + (3.0 * g), 0.0, 180.0);
            } else if (j->buttons[2] == 1 && j->buttons[1] == 0) { // Quadrado/X
                ang_s2_ = std::clamp(ang_s2_ - (3.0 * g), 0.0, 180.0);
            }
        }
        pubF64(pub_s2_, ang_s2_);
    }

    std::mutex mtx_;
    sensor_msgs::msg::Joy::SharedPtr joy_;
    bool lt_ok_ = false, rt_ok_ = false;
    double val_s0_ = 90.0;
    double ang_s1_ = 90.0;
    double ang_s2_ = 90.0;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_elev_, pub_s0_, pub_s1_, pub_s2_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControleNode>());
    rclcpp::shutdown();
    return 0;
}

