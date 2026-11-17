#!/usr/bin/env python3
import os
import sys
import select
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# =====================================================================
# MAPA DE PONTOS DA ARENA E SEQUÊNCIA SORTEADA (MÓDULO DE NAVEGAÇÃO)
# =====================================================================
PONTOS_ARENA = {
    'A': (-0.44, -1.22,  90.0),
    'B': ( 1.01, -3.02,   0.0),
    'C': (-0.82, -0.04, 180.0),
    'D': (-0.82,  0.05, 270.0),
}

# Sequência sorteada pelos avaliadores (Ex: ['B', 'C', 'A', 'D', 'C', 'B'])
SEQUENCIA_SORTEADA = ['A', 'B', 'C', 'D']


def criar_pose(x, y, yaw_graus):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = Time().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)

    rad = math.radians(yaw_graus)
    pose.pose.orientation.z = math.sin(rad / 2.0)
    pose.pose.orientation.w = math.cos(rad / 2.0)
    return pose


class PainelAuxiliar(Node):
    def __init__(self):
        super().__init__('nodo_auxiliar_prova')
        self.cmd_vel_pub        = self.create_publisher(Twist, '/cmd_vel', 10)
        self.init_pose_pub      = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.script_rodando_pub = self.create_publisher(Bool, '/painel/script_rodando', 10)
        self.laranja_pub        = self.create_publisher(Bool, '/painel/piscar_laranja', 10)

        self.script_ativo           = False

        self.emergencia_pressionada = False
        self.emergencia_ocorreu     = False

        self.stop_pressionado       = False
        self.stop_pausado           = False

        self.reset_ant              = False
        self.reset_clicado          = False

        self.start_ant              = False
        self.start_clicado          = False

        self.create_subscription(Bool, '/painel/emerg_pressed', self.emerg_cb, 10)
        self.create_subscription(Bool, '/painel/stop_pressed', self.stop_cb, 10)
        self.create_subscription(Bool, '/painel/reset_pressed', self.reset_cb, 10)
        self.create_subscription(Bool, '/painel/start_pressed', self.start_cb, 10)

        self.create_timer(0.1, self.publicar_estado_script)

    def forcar_pose_zero_amcl(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = Time().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.02
        self.init_pose_pub.publish(msg)

    def publicar_estado_script(self):
        if not rclpy.ok():
            return
        em_execucao = (
            self.script_ativo
            and not self.emergencia_pressionada
            and not self.emergencia_ocorreu
            and not self.stop_pausado
        )
        self.set_bool(self.script_rodando_pub, em_execucao)

    def emerg_cb(self, msg: Bool):
        self.emergencia_pressionada = msg.data
        if msg.data:
            self.emergencia_ocorreu = True
            self.script_ativo       = False
            self.cmd_vel_pub.publish(Twist())
            self.set_bool(self.script_rodando_pub, False)
            self.set_bool(self.laranja_pub, False)

    def stop_cb(self, msg: Bool):
        self.stop_pressionado = msg.data
        if msg.data and not self.stop_pausado:
            self.stop_pausado = True
            self.cmd_vel_pub.publish(Twist())
            self.set_bool(self.script_rodando_pub, False)
            self.set_bool(self.laranja_pub, False)

    def reset_cb(self, msg: Bool):
        apertado = msg.data
        if apertado and not self.reset_ant:
            self.reset_clicado = True
        self.reset_ant = apertado

    def start_cb(self, msg: Bool):
        apertado = msg.data
        if apertado and not self.start_ant:
            self.start_clicado = True
        self.start_ant = apertado

    def set_bool(self, pub, valor: bool):
        if not rclpy.ok():
            return
        msg = Bool()
        msg.data = valor
        pub.publish(msg)

    def parar_tudo_imediatamente(self, nav: BasicNavigator):
        self.cmd_vel_pub.publish(Twist())
        self.set_bool(self.script_rodando_pub, False)
        self.set_bool(self.laranja_pub, False)
        try:
            if nav.goal_handle is not None:
                nav.goal_handle.cancel_goal_async()
                nav.goal_handle = None
                nav.result_future = None
        except Exception:
            pass
        self.cmd_vel_pub.publish(Twist())

    def reiniciar_codigo(self, nav: BasicNavigator):
        print("\n>>> [REINÍCIO] A reiniciar a sequência completa desde o início...")
        self.script_ativo = False
        self.parar_tudo_imediatamente(nav)
        time.sleep(0.1)
        self.destroy_node()
        rclpy.shutdown()
        os.execv(sys.executable, [sys.executable] + sys.argv)

    def tratar_reposicionamento_emergencia(self, nav: BasicNavigator) -> bool:
        if not (self.emergencia_pressionada or self.emergencia_ocorreu):
            return False

        self.script_ativo = False
        self.parar_tudo_imediatamente(nav)
        self.reset_clicado = False
        self.start_clicado = False

        print("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print("!!! [EMERGÊNCIA / REPOSICIONAMENTO] Robô parado (LED VERMELHO ACESO).")
        print("!!! 1. Reposicione o robô manualmente na ÁREA DE INÍCIO (0, 0, 0)")
        print("!!! 2. Desacione (levante) o botão de EMERGÊNCIA (Pino 11)")
        print("!!! 3. (Opcional) Pressione RESET (Pino 9)")
        print("!!! 4. Pressione START (Pino 8) uma única vez para retomar!")
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

        reset_ja_apertado   = False
        preparado_pos_emerg = False

        while rclpy.ok():
            self.cmd_vel_pub.publish(Twist())
            self.set_bool(self.script_rodando_pub, False)
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.emergencia_pressionada:
                self.start_clicado  = False
                self.reset_clicado  = False
                preparado_pos_emerg = False
                continue

            if not preparado_pos_emerg:
                preparado_pos_emerg = True
                self.forcar_pose_zero_amcl()

            if self.reset_clicado:
                self.reset_clicado = False
                if not reset_ja_apertado:
                    reset_ja_apertado = True
                    self.forcar_pose_zero_amcl()
                    print(">>> [PASSO 4 - RESET OPCIONAL] Pose (0,0,0) confirmada! Pressione START (Pino 8) para partir.")
                else:
                    self.reiniciar_codigo(nav)

            if not self.stop_pressionado:
                if self.start_clicado:
                    self.start_clicado = False
                    break

                if select.select([sys.stdin], [], [], 0.0)[0]:
                    sys.stdin.readline()
                    break

        self.forcar_pose_zero_amcl()
        self.emergencia_ocorreu = False
        self.stop_pausado       = False
        self.script_ativo       = True
        self.set_bool(self.script_rodando_pub, True)
        print(">>> [START APÓS REPOSICIONAMENTO] Partindo imediatamente (LED VERDE ACESO)...")
        return True

    def verificar_stop_pausa(self, nav: BasicNavigator) -> bool:
        if not self.stop_pausado:
            return False

        self.parar_tudo_imediatamente(nav)
        self.start_clicado = False
        self.reset_clicado = False

        print("\n=====================================================================")
        print(">>> [STOP - PAUSADO] Robô parado (LED STOPPED 18 + LED STOP 15 ACESOS)!")
        print(">>> Pressione START (Pino 8) para RETOMAR ou RESET (Pino 9) para REINICIAR.")
        print("=====================================================================")

        while rclpy.ok():
            self.cmd_vel_pub.publish(Twist())
            self.set_bool(self.script_rodando_pub, False)
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.emergencia_pressionada or self.emergencia_ocorreu:
                return self.tratar_reposicionamento_emergencia(nav)

            if self.reset_clicado:
                self.reset_clicado = False
                self.reiniciar_codigo(nav)

            if not self.stop_pressionado:
                if self.start_clicado:
                    self.start_clicado = False
                    break

                if select.select([sys.stdin], [], [], 0.0)[0]:
                    sys.stdin.readline()
                    break

        self.stop_pausado = False
        self.script_ativo = True
        self.set_bool(self.script_rodando_pub, True)
        print(">>> [RETOMANDO] Voltando à navegação imediatamente (LED RUNNING 19 Verde ACESO)...")
        return True

    def aguardar_botao_start_inicial(self, nav: BasicNavigator):
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.01)

        self.script_ativo  = False
        self.stop_pausado  = False
        self.start_clicado = False
        self.reset_clicado = False
        self.set_bool(self.script_rodando_pub, False)
        self.set_bool(self.laranja_pub, False)
        self.forcar_pose_zero_amcl()

        sequencia_str = " -> ".join(SEQUENCIA_SORTEADA)
        print("\n=====================================================================")
        print(f">>> SEQUÊNCIA CONFIGURADA: {sequencia_str}")
        print(">>> ROBÔ PARADO NA ÁREA DE START (LED VERMELHO 18 + LED START 17 ACESOS)")
        print(">>> Pressione START (Pino 8) ou ENTER para iniciar imediatamente!")
        print("=====================================================================")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.emergencia_pressionada:
                self.start_clicado = False
                continue

            if self.emergencia_ocorreu:
                self.emergencia_ocorreu = False
                self.forcar_pose_zero_amcl()

            if self.reset_clicado:
                self.reset_clicado = False
                self.forcar_pose_zero_amcl()
                print(">>> [RESET] Pose inicial (0,0,0) confirmada! Pronto para START...")

            if self.start_clicado and not self.stop_pressionado:
                self.start_clicado = False
                break

            if select.select([sys.stdin], [], [], 0.0)[0]:
                sys.stdin.readline()
                if not (self.emergencia_pressionada or self.stop_pressionado):
                    break

        self.script_ativo = True
        self.set_bool(self.script_rodando_pub, True)
        print(">>> START ACIONADO! Partindo imediatamente (LED VERDE 19 ACESO)...")

    def piscar_reset_5s_validacao(self, nav: BasicNavigator, nome_ponto: str, passo_num: int):
        print(f">>> [VALIDAÇÃO] Ponto {nome_ponto} ({passo_num}/{len(SEQUENCIA_SORTEADA)}): Piscando LED de RESET (Pino 16) por 5 segundos...")
        self.cmd_vel_pub.publish(Twist())
        self.set_bool(self.laranja_pub, True)
        acumulado = 0.0
        passo = 0.02

        while acumulado < 5.0 and rclpy.ok():
            t0 = time.time()
            self.cmd_vel_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=passo)

            if self.emergencia_pressionada or self.emergencia_ocorreu:
                self.set_bool(self.laranja_pub, False)
                return False

            if self.reset_clicado:
                self.reiniciar_codigo(nav)

            if self.stop_pausado:
                self.verificar_stop_pausa(nav)
                self.set_bool(self.laranja_pub, True)

            dt = time.time() - t0
            if dt < passo:
                time.sleep(passo - dt)
            acumulado += max(dt, passo)

        self.set_bool(self.laranja_pub, False)
        self.set_bool(self.script_rodando_pub, True)
        print(f">>> [VALIDADO] Ponto {nome_ponto} concluído!")
        return True


def main():
    rclpy.init()
    nav = BasicNavigator()
    painel = PainelAuxiliar()

    try:
        for _ in range(10):
            rclpy.spin_once(painel, timeout_sec=0.01)

        pose_inicial = criar_pose(0.0, 0.0, 0.0)
        nav.setInitialPose(pose_inicial)

        print(">>> A aguardar que o Nav2 fique ativo...")
        nav.waitUntilNav2Active()

        nav.clearAllCostmaps()
        painel.aguardar_botao_start_inicial(nav)

        for idx, letra_ponto in enumerate(SEQUENCIA_SORTEADA, start=1):
            x, y, yaw = PONTOS_ARENA[letra_ponto]

            while rclpy.ok():
                if painel.emergencia_pressionada or painel.emergencia_ocorreu:
                    painel.tratar_reposicionamento_emergencia(nav)

                if painel.stop_pausado:
                    painel.verificar_stop_pausa(nav)

                print(f"\n>>> [ETAPA {idx}/{len(SEQUENCIA_SORTEADA)}] Indo para o Ponto '{letra_ponto}': X={x}, Y={y}, Yaw={yaw}°")
                meta = criar_pose(x, y, yaw)
                nav.goToPose(meta)

                interrompido = False
                print(">>> A navegar...", end="", flush=True)
                ultimo_ponto_print = time.time()

                while not nav.isTaskComplete() and rclpy.ok():
                    rclpy.spin_once(painel, timeout_sec=0.01)

                    if painel.emergencia_pressionada or painel.emergencia_ocorreu:
                        painel.tratar_reposicionamento_emergencia(nav)
                        interrompido = True
                        break

                    if painel.reset_clicado:
                        painel.reiniciar_codigo(nav)

                    if painel.stop_pausado:
                        painel.verificar_stop_pausa(nav)
                        interrompido = True
                        break

                    if time.time() - ultimo_ponto_print >= 0.5:
                        print(".", end="", flush=True)
                        ultimo_ponto_print = time.time()

                if interrompido:
                    continue

                print("\n>>> Ciclo de navegação encerrado!")
                resultado = nav.getResult()

                if resultado == TaskResult.SUCCEEDED:
                    print(f">>> Robô posicionado no Ponto '{letra_ponto}'!")
                    validou = painel.piscar_reset_5s_validacao(nav, letra_ponto, idx)
                    if not validou:
                        continue
                    break
                else:
                    if painel.emergencia_pressionada or painel.emergencia_ocorreu:
                        continue
                    if painel.stop_pausado:
                        continue

                    print(f">>> Replanejando rota alternativa para o Ponto '{letra_ponto}'...")
                    time.sleep(0.2)
                    continue

        print("\n=====================================================================")
        print(">>> PERCURSO FINALIZADO COM SUCESSO DENTRO DA ÁREA DELIMITADA!")
        print(">>> Robô Parado: LED VERMELHO (18) ACESO e LED VERDE (19) APAGADO.")
        print("=====================================================================")

    finally:
        painel.script_ativo = False
        if rclpy.ok():
            painel.set_bool(painel.script_rodando_pub, False)
            painel.set_bool(painel.laranja_pub, False)
            painel.cmd_vel_pub.publish(Twist())
            time.sleep(0.1)
            painel.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()