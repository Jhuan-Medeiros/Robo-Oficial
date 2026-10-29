#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <algorithm>

using namespace std::chrono_literals;

class OMSNode : public rclcpp::Node {
public:
    OMSNode() : Node("oms_node"), ocupado_(false), aguardar_distancia_(false), encoder_atual_(0),
                limit_fwd_ativado_(false), limit_rev_ativado_(false) {
        this->declare_parameter<double>("garra_aberta", 40.0);
        this->declare_parameter<double>("pulso_baixo", 45.0);
        this->declare_parameter<double>("pulso_reto", 90.0);
        this->declare_parameter<double>("pulso_cima", 135.0);
        this->declare_parameter<double>("extensor_parado", 90.0);
        this->declare_parameter<double>("extensor_frente", 140.0);
        this->declare_parameter<double>("extensor_tras", 40.0);
        this->declare_parameter<int>("tempo_extensor_ms", 1000);
        this->declare_parameter<double>("distancia_pegar", 0.30);

        this->declare_parameter<int>("objeto_chao_altura_ticks", 0);
        this->declare_parameter<double>("objeto_chao_garra_fechada", 130.0);
        this->declare_parameter<double>("objeto_chao_vel_motor", -0.4);

        this->declare_parameter<int>("objeto_andar1_altura_ticks", 500);
        this->declare_parameter<double>("objeto_andar1_garra_fechada", 120.0);
        this->declare_parameter<double>("objeto_andar1_vel_motor", 0.4);

        this->declare_parameter<int>("objeto_andar2_altura_ticks", 1200);
        this->declare_parameter<double>("objeto_andar2_garra_fechada", 110.0);
        this->declare_parameter<double>("objeto_andar2_vel_motor", 0.4);

        pub_extensor_ = this->create_publisher<std_msgs::msg::Float64>("/servo0/cmd", 10);
        pub_pulso_    = this->create_publisher<std_msgs::msg::Float64>("/servo1/cmd", 10);
        pub_garra_    = this->create_publisher<std_msgs::msg::Float64>("/servo2/cmd", 10);
        pub_elevador_ = this->create_publisher<std_msgs::msg::Float64>("/titan/m_3/cmd", 10);
        pub_cmd_vel_  = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_status_   = this->create_publisher<std_msgs::msg::String>("/oms/status", 10);

        sub_limit_fwd_ = this->create_subscription<std_msgs::msg::Bool>(
            "/titan/m_3/limit_fwd", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                limit_fwd_ativado_ = msg->data;
            });
        
        sub_limit_rev_ = this->create_subscription<std_msgs::msg::Bool>(
            "/titan/m_3/limit_rev", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                limit_rev_ativado_ = msg->data;
            });

        sub_comando_ = this->create_subscription<std_msgs::msg::String>(
            "/oms/comando", 10,
            std::bind(&OMSNode::comandoCallback, this, std::placeholders::_1));

        sub_encoder_ = this->create_subscription<std_msgs::msg::Float64>(
            "/titan/m_3/encoder", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                encoder_atual_ = static_cast<int>( std::round(msg->data / 0.00022165) );
            });

        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&OMSNode::scanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), ">>> OMS Node Pronto (Controle + Automatico + Fim de Curso)! <<<");
    }

private:
    void enviarServo(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub, double valor) {
        std_msgs::msg::Float64 msg;
        msg.data = valor;
        pub->publish(msg);
    }

    void enviarElevador(double potencia) {
        std_msgs::msg::Float64 msg;
        msg.data = potencia;
        pub_elevador_->publish(msg);
    }

    void moverElevadorParaTicks(int ticks_alvo, double potencia_base) {
        rclcpp::Rate taxa(50);
        int tolerancia = 25;
        int timeout_loops = 200; // Seguranca: max 4 segundos

        while (rclcpp::ok() && timeout_loops-- > 0) {
            // Trava de seguranca de fim de curso
            if ((potencia_base > 0 && limit_fwd_ativado_.load()) || 
                (potencia_base < 0 && limit_rev_ativado_.load())) {
                RCLCPP_WARN(this->get_logger(), "Fim de curso atingido! Parando elevador.");
                break;
            }

            int erro = ticks_alvo - encoder_atual_.load();
            if (std::abs(erro) <= tolerancia) {
                break;
            }
            double potencia = (erro > 0) ? std::abs(potencia_base) : -std::abs(potencia_base);
            enviarElevador(potencia);
            taxa.sleep();
        }
        enviarElevador(0.0);
    }

    void moverExtensorTempo(double valor_movimento, double valor_parado, int tempo_ms) {
        enviarServo(pub_extensor_, valor_movimento);
        std::this_thread::sleep_for(std::chrono::milliseconds(tempo_ms));
        enviarServo(pub_extensor_, valor_parado);
    }

    void publicarStatus(const std::string &status) {
        std_msgs::msg::String msg;
        msg.data = status;
        pub_status_->publish(msg);
    }

    void comandoCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string raw_cmd = msg->data;
        std::string acao = "PEGAR";
        std::string tipo_obj = "CHAO";

        size_t comma = raw_cmd.find(',');
        if (comma != std::string::npos) {
            acao = raw_cmd.substr(0, comma);
            tipo_obj = raw_cmd.substr(comma + 1);
        } else {
            acao = raw_cmd;
        }

        std::transform(acao.begin(), acao.end(), acao.begin(), ::toupper);
        std::transform(tipo_obj.begin(), tipo_obj.end(), tipo_obj.begin(), ::toupper);

        if (acao == "ELEV_SUBIR_CONT") {
            enviarElevador(0.5);
            return;
        } else if (acao == "ELEV_DESCER_CONT") {
            enviarElevador(-0.4);
            return;
        } else if (acao == "ELEV_PARAR") {
            enviarElevador(0.0);
            return;
        } else if (acao == "EXT_AVANCAR_CONT") {
            enviarServo(pub_extensor_, this->get_parameter("extensor_frente").as_double());
            return;
        } else if (acao == "EXT_RECUAR_CONT") {
            enviarServo(pub_extensor_, this->get_parameter("extensor_tras").as_double());
            return;
        } else if (acao == "EXT_PARAR") {
            enviarServo(pub_extensor_, this->get_parameter("extensor_parado").as_double());
            return;
        }

        if (acao == "AUTO_PEGAR") {
            aguardar_distancia_ = true;
            RCLCPP_INFO(this->get_logger(), "Modo AUTO_PEGAR ativado para objeto [%s]!", tipo_obj.c_str());
            return;
        }

        if (ocupado_.load()) {
            RCLCPP_WARN(this->get_logger(), "OMS ocupado! Comando ignorado: %s", acao.c_str());
            return;
        }

        std::thread(&OMSNode::executarRotina, this, acao, tipo_obj).detach();
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        if (!aguardar_distancia_.load() || ocupado_.load()) return;

        double dist_alvo = this->get_parameter("distancia_pegar").as_double();
        int total_feixes = scan->ranges.size();
        if (total_feixes == 0) return;

        int centro = total_feixes / 2;
        int janela = std::max(1, total_feixes / 36);
        double menor_dist = 999.0;

        for (int i = centro - janela; i <= centro + janela; ++i) {
            if (i >= 0 && i < total_feixes) {
                float r = scan->ranges[i];
                if (std::isfinite(r) && r > scan->range_min && r < menor_dist) {
                    menor_dist = r;
                }
            }
        }

        if (menor_dist <= dist_alvo) {
            aguardar_distancia_ = false;
            RCLCPP_INFO(this->get_logger(), "Objeto detectado a %.2f m! Parando robo e acionando PEGAR.", menor_dist);

            geometry_msgs::msg::Twist stop_msg;
            pub_cmd_vel_->publish(stop_msg);

            std::thread(&OMSNode::executarRotina, this, "PEGAR", "CHAO").detach();
        }
    }

    void executarRotina(std::string acao, std::string tipo_obj) {
        ocupado_ = true;
        publicarStatus("OCUPADO:" + acao + "_" + tipo_obj);
        RCLCPP_INFO(this->get_logger(), "Iniciando [%s] para objeto [%s]", acao.c_str(), tipo_obj.c_str());

        double garra_aberta   = this->get_parameter("garra_aberta").as_double();
        double pulso_baixo    = this->get_parameter("pulso_baixo").as_double();
        double pulso_reto     = this->get_parameter("pulso_reto").as_double();
        double pulso_cima     = this->get_parameter("pulso_cima").as_double();
        double ext_parado     = this->get_parameter("extensor_parado").as_double();
        double ext_frente     = this->get_parameter("extensor_frente").as_double();
        double ext_tras       = this->get_parameter("extensor_tras").as_double();
        int tempo_ext         = this->get_parameter("tempo_extensor_ms").as_int();

        int ticks_alvo = this->get_parameter("objeto_chao_altura_ticks").as_int();
        double garra_fechada = this->get_parameter("objeto_chao_garra_fechada").as_double();
        double vel_motor = this->get_parameter("objeto_chao_vel_motor").as_double();

        if (tipo_obj == "ANDAR1") {
            ticks_alvo = this->get_parameter("objeto_andar1_altura_ticks").as_int();
            garra_fechada = this->get_parameter("objeto_andar1_garra_fechada").as_double();
            vel_motor = this->get_parameter("objeto_andar1_vel_motor").as_double();
        } else if (tipo_obj == "ANDAR2") {
            ticks_alvo = this->get_parameter("objeto_andar2_altura_ticks").as_int();
            garra_fechada = this->get_parameter("objeto_andar2_garra_fechada").as_double();
            vel_motor = this->get_parameter("objeto_andar2_vel_motor").as_double();
        }

        if (acao == "ABRIR") {
            enviarServo(pub_garra_, garra_aberta);
        } else if (acao == "FECHAR") {
            enviarServo(pub_garra_, garra_fechada);
        } else if (acao == "PULSO_BAIXO") {
            enviarServo(pub_pulso_, pulso_baixo);
        } else if (acao == "PULSO_RETO") {
            enviarServo(pub_pulso_, pulso_reto);
        } else if (acao == "PULSO_CIMA") {
            enviarServo(pub_pulso_, pulso_cima);
        } else if (acao == "HOME") {
            enviarServo(pub_extensor_, ext_parado);
            enviarServo(pub_garra_, garra_aberta);
            enviarServo(pub_pulso_, pulso_cima);
            moverElevadorParaTicks(0, 0.5);
            std::this_thread::sleep_for(500ms);
        } else if (acao == "PEGAR") {
            enviarServo(pub_garra_, garra_aberta);
            enviarServo(pub_pulso_, pulso_baixo);
            std::this_thread::sleep_for(500ms);

            moverElevadorParaTicks(ticks_alvo, vel_motor);
            moverExtensorTempo(ext_frente, ext_parado, tempo_ext);

            enviarServo(pub_garra_, garra_fechada);
            std::this_thread::sleep_for(700ms);

            enviarServo(pub_pulso_, pulso_reto);
            moverElevadorParaTicks(0, 0.5);

            moverExtensorTempo(ext_tras, ext_parado, tempo_ext);
        } else if (acao == "SOLTAR") {
            moverExtensorTempo(ext_frente, ext_parado, tempo_ext);
            enviarServo(pub_pulso_, pulso_baixo);
            std::this_thread::sleep_for(500ms);

            enviarServo(pub_garra_, garra_aberta);
            std::this_thread::sleep_for(600ms);

            moverExtensorTempo(ext_tras, ext_parado, tempo_ext);
            enviarServo(pub_pulso_, pulso_cima);
        } else {
            RCLCPP_WARN(this->get_logger(), "Comando desconhecido: [%s]", acao.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Acao [%s] concluida!", acao.c_str());
        publicarStatus("PRONTO");
        ocupado_ = false;
    }

    std::atomic<bool> ocupado_;
    std::atomic<bool> aguardar_distancia_;
    std::atomic<int> encoder_atual_;
    std::atomic<bool> limit_fwd_ativado_;
    std::atomic<bool> limit_rev_ativado_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_extensor_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pulso_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_garra_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_elevador_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_comando_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_encoder_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_limit_fwd_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_limit_rev_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OMSNode>());
    rclcpp::shutdown();
    return 0;
}