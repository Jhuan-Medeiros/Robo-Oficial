#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

#include <vector>
#include <cmath>
#include <string>

class ControleNode : public rclcpp::Node {
public:
    ControleNode() : Node("controle_node"), garra_fechada_estado_(false) {
        this->declare_parameter<double>("vel_linear_max", 0.35);
        this->declare_parameter<double>("vel_angular_max", 1.2);
        this->declare_parameter<double>("multiplicador_turbo", 1.8);

        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_oms_     = this->create_publisher<std_msgs::msg::String>("/oms/comando", 10);

        sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&ControleNode::joyCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), ">>> Controle Node Iniciado! Use os analogicos e botoes do Gamepad <<<");
    }

private:
    double aplicarDeadzone(double valor, double limite = 0.12) {
        return (std::abs(valor) < limite) ? 0.0 : valor;
    }

    void enviarComandoOMS(const std::string &cmd) {
        std_msgs::msg::String msg;
        msg.data = cmd;
        pub_oms_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "[Controle -> OMS]: %s", cmd.c_str());
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {
        if (joy->axes.size() < 4 || joy->buttons.size() < 6) return;

        if (botoes_ant_.empty()) {
            botoes_ant_ = joy->buttons;
            eixos_ant_  = joy->axes;
        }

        // 1. MOVIMENTACAO DO ROBO (Analogico Esquerdo = X/Y | Analogico Direito = Giro)
        double v_lin_max = this->get_parameter("vel_linear_max").as_double();
        double v_ang_max = this->get_parameter("vel_angular_max").as_double();

        // Botao RB (indice 5) funciona como Turbo quando segurado
        if (joy->buttons[5] == 1) {
            double turbo = this->get_parameter("multiplicador_turbo").as_double();
            v_lin_max *= turbo;
            v_ang_max *= turbo;
        }

        // No Logitech F310 / Gamepad padrao Linux:
        // axes[1] = Analogico Esq Vertical (+1 cima), axes[0] = Analogico Esq Horizontal (+1 esquerda)
        // axes[2] ou axes[3] = Analogico Direito Horizontal (giro)
        double eixo_x   = aplicarDeadzone(joy->axes[1]);
        double eixo_y   = aplicarDeadzone(joy->axes[0]);
        double eixo_rot = aplicarDeadzone(joy->axes[3]);
        if (std::abs(eixo_rot) < 0.01 && joy->axes.size() > 2) {
            // Caso o controle esteja no modo DirectInput onde o giro fica no axes[2]
            if (std::abs(joy->axes[2]) > 0.12 && std::abs(joy->axes[2] - 1.0) > 0.05) {
                eixo_rot = aplicarDeadzone(joy->axes[2]);
            }
        }

        geometry_msgs::msg::Twist twist;
        twist.linear.x  = eixo_x * v_lin_max;
        twist.linear.y  = eixo_y * v_lin_max;
        twist.angular.z = eixo_rot * v_ang_max;
        pub_cmd_vel_->publish(twist);

        // 2. BOTOES DE ACAO AUTOMATICA DO OMS (Detector de borda de subida)
        // Botao A (0) -> PEGAR,CHAO
        if (joy->buttons[0] == 1 && botoes_ant_[0] == 0) {
            enviarComandoOMS("PEGAR,CHAO");
        }
        // Botao B (1) -> SOLTAR
        if (joy->buttons[1] == 1 && botoes_ant_[1] == 0) {
            enviarComandoOMS("SOLTAR");
        }
        // Botao X (2) -> PEGAR,ANDAR1
        if (joy->buttons[2] == 1 && botoes_ant_[2] == 0) {
            enviarComandoOMS("PEGAR,ANDAR1");
        }
        // Botao Y (3) -> HOME
        if (joy->buttons[3] == 1 && botoes_ant_[3] == 0) {
            enviarComandoOMS("HOME");
        }

        // 3. CONTROLE MANUAL DA GARRA E PULSO
        // Botao LB (4) -> Alterna Abrir / Fechar Garra
        if (joy->buttons[4] == 1 && botoes_ant_[4] == 0) {
            if (garra_fechada_estado_) {
                enviarComandoOMS("ABRIR");
                garra_fechada_estado_ = false;
            } else {
                enviarComandoOMS("FECHAR");
                garra_fechada_estado_ = true;
            }
        }
        // Botoes 6 e 7 (LT/RT ou Back/Start dependendo do modo) -> Pulso Baixo / Cima
        if (joy->buttons.size() > 6 && joy->buttons[6] == 1 && botoes_ant_[6] == 0) {
            enviarComandoOMS("PULSO_BAIXO");
        }
        if (joy->buttons.size() > 7 && joy->buttons[7] == 1 && botoes_ant_[7] == 0) {
            enviarComandoOMS("PULSO_CIMA");
        }

        // 4. CONTROLE MANUAL DO ELEVADOR E EXTENSOR PELAS SETINHAS (D-PAD)
        // Normalmente D-Pad Horizontal = axes[6] e D-Pad Vertical = axes[7]
        if (joy->axes.size() >= 8) {
            // Elevador (D-Pad Vertical - axes[7])
            if (joy->axes[7] > 0.5 && eixos_ant_[7] <= 0.5) {
                enviarComandoOMS("ELEV_SUBIR_CONT");
            } else if (joy->axes[7] < -0.5 && eixos_ant_[7] >= -0.5) {
                enviarComandoOMS("ELEV_DESCER_CONT");
            } else if (std::abs(joy->axes[7]) < 0.1 && std::abs(eixos_ant_[7]) >= 0.5) {
                enviarComandoOMS("ELEV_PARAR");
            }

            // Extensor 360 (D-Pad Horizontal - axes[6])
            if (joy->axes[6] < -0.5 && eixos_ant_[6] >= -0.5) { // Direita
                enviarComandoOMS("EXT_AVANCAR_CONT");
            } else if (joy->axes[6] > 0.5 && eixos_ant_[6] <= 0.5) { // Esquerda
                enviarComandoOMS("EXT_RECUAR_CONT");
            } else if (std::abs(joy->axes[6]) < 0.1 && std::abs(eixos_ant_[6]) >= 0.5) {
                enviarComandoOMS("EXT_PARAR");
            }
        }

        botoes_ant_ = joy->buttons;
        eixos_ant_  = joy->axes;
    }

    bool garra_fechada_estado_;
    std::vector<int32_t> botoes_ant_;
    std::vector<float> eixos_ant_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_oms_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControleNode>());
    rclcpp::shutdown();
    return 0;
}
