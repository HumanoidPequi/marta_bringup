#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
from diagnostic_updater import Updater, FunctionDiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus

# Variáveis globais para armazenar o estado da IMU
last_imu_msg_time = None
imu_update_frequency = 50.0  # Frequência esperada da IMU (em Hz)

def check_imu_status(stat):
    """
    Verifica o status da IMU, incluindo frequência de atualização e última mensagem recebida.
    """
    global last_imu_msg_time

    # Checa se a IMU está publicando mensagens
    if last_imu_msg_time is None:
        stat.summary(DiagnosticStatus.ERROR, "Nenhuma mensagem da IMU recebida.")
        return stat

    # Calcula o tempo desde a última mensagem
    time_since_last_msg = rospy.get_time() - last_imu_msg_time
    if time_since_last_msg > (1.0 / imu_update_frequency * 2):  # Verifica se a frequência caiu
        stat.summary(DiagnosticStatus.WARN, f"IMU esta atrasada: ultima mensagem ha {time_since_last_msg:.2f} s")
    else:
        stat.summary(DiagnosticStatus.OK, "IMU operando normalmente")

    # Adiciona informações detalhadas
    stat.add("Ultima mensagem recebida", f"{time_since_last_msg:.2f} s atras")
    stat.add("Frequencia esperada", f"{imu_update_frequency:.2f} Hz")
    return stat

def imu_callback(msg):
    """
    Callback para mensagens de IMU.
    """
    global last_imu_msg_time
    last_imu_msg_time = rospy.get_time()  # Atualiza o timestamp da última mensagem recebida

def main():
    rospy.init_node('imu_diagnostics')

    # Inicia o diagnostic_updater
    updater = Updater()
    updater.setHardwareID("Marta_IMU_Sensor")

    # Adiciona a tarefa de diagnóstico corretamente
    updater.add("IMU Status", check_imu_status)

    # Inscreve-se no tópico da IMU
    imu_topic = rospy.get_param("~imu_topic", "/marta/imu")  # Tópico atualizado para /marta/imu
    rospy.Subscriber(imu_topic, Imu, imu_callback)

    # Frequência de publicação dos diagnósticos
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        updater.update()
        rate.sleep()

if __name__ == "__main__":
    main()

