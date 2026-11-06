#!/usr/bin/env python3
import time
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def criar_pose(navigator, x, y, yaw_graus):
    """Função auxiliar para converter coordenadas e ângulos em poses do ROS 2."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    
    # Conversão de graus para quaternions no eixo Z
    rad = math.radians(yaw_graus)
    pose.pose.orientation.z = math.sin(rad / 2.0)
    pose.pose.orientation.w = math.cos(rad / 2.0)
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()

    print(">>> A aguardar que o Nav2 fique ativo...")
    nav.waitUntilNav2Active()

    # 1. Definir a pose inicial onde o robô está posicionado na pista (X=0, Y=0, Ângulo=0)
    pose_inicial = criar_pose(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(pose_inicial)
    
    # Pausa de segurança para o AMCL assimilar a pose inicial
    time.sleep(2.0)

    # 2. Lista com as coordenadas dos 6 locais da prova: (X, Y, Orientação Yaw em graus)
    # Substitui estes valores pelas coordenadas reais exatas que tiraste no RViz!
    locais_prova = [
        (1.00,  0.50,   0.0),   # Local 1 (Ex: Recolha do item 1)
        (2.00,  1.00,  90.0),   # Local 2 (Ex: Depósito do item 1)
        (1.50, -0.50, 180.0),   # Local 3
        (0.50, -1.00, 270.0),   # Local 4
        (-1.00, 0.00,   0.0),   # Local 5
        (-2.00, 1.00, 180.0)    # Local 6
    ]

    # Criar o publicador para a garra/servos (exemplo no /servo0/cmd)
    node = rclpy.create_node('nodo_auxiliar_servo')
    servo_pub = node.create_publisher(Float64, '/servo0/cmd', 10)

    for i, (x, y, yaw) in enumerate(locais_prova, start=1):
        print(f"\n>>> A enviar meta para o Ponto {i}: X={x}, Y={y}")
        meta = criar_pose(nav, x, y, yaw)
        nav.goToPose(meta)

        # Aguardar enquanto o robô se desloca (com feedback visual no terminal)
        print(">>> A aguardar o robô chegar ao destino...", end="", flush=True)
        while not nav.isTaskComplete():
            print(".", end="", flush=True)
            time.sleep(1.0)

        print("\n>>> Ciclo de tarefa concluído!")
        resultado = nav.getResult()
        
        if resultado == TaskResult.SUCCEEDED:
            print(f">>> Chegou com sucesso ao Ponto {i}!")
            
            # Acionamento automático da garra ao chegar
            msg_servo = Float64()
            msg_servo.data = 40.0  # Ângulo de atuação
            servo_pub.publish(msg_servo)
            time.sleep(1.5)        # Pausa para garantir que a garra atua
            
        else:
            print(f">>> Falha ou cancelamento ao alcançar o Ponto {i}. A limpar costmaps e a tentar recuperar...")
            nav.clearAllCostmaps()

    print("\n>>> Rotina dos 6 pontos concluída com sucesso!")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()