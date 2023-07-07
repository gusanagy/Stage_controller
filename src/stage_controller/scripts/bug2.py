#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Bug2Algorithm:
    def __init__(self):
        rospy.init_node('bug2_algorithm')  # Inicializa o nó ROS

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        self.robot_velocity = Twist()  # Velocidade linear e angular do robô
        self.is_obstacle_detected = False  # Flag para indicar a detecção de obstáculos
        self.distance_to_obstacle = 0.0  # Distância até o obstáculo mais próximo

    def laser_callback(self, data):
        # Função de retorno de chamada para lidar com os dados do laser
        # Verifica se há obstáculos e atualiza as variáveis apropriadas
        self.is_obstacle_detected = min(data.ranges) < 0.5
        self.distance_to_obstacle = min(data.ranges)

    def move_forward(self, distance):
        # Move o robô para a frente em uma determinada distância
        initial_distance = 0.0
        rate = rospy.Rate(10)  # Taxa de atualização do loop (10 Hz)

        while initial_distance < distance:
            self.robot_velocity.linear.x = 0.2  # Velocidade linear
            self.robot_velocity.angular.z = 0.0  # Velocidade angular

            self.velocity_publisher.publish(self.robot_velocity)
            rate.sleep()

            initial_distance += 0.2 / 10  # Atualiza a distância percorrida

        # Após alcançar a distância desejada, pare o robô
        self.robot_velocity.linear.x = 0.0
        self.velocity_publisher.publish(self.robot_velocity)

    def turn(self, angle):
        # Faz o robô girar em um determinado ângulo
        initial_angle = 0.0
        rate = rospy.Rate(10)  # Taxa de atualização do loop (10 Hz)

        while initial_angle < angle:
            self.robot_velocity.linear.x = 0.0  # Velocidade linear
            self.robot_velocity.angular.z = 0.2  # Velocidade angular

            self.velocity_publisher.publish(self.robot_velocity)
            rate.sleep()

            initial_angle += 0.2 / 10  # Atualiza o ângulo girado

        # Após atingir o ângulo desejado, pare o robô
        self.robot_velocity.angular.z = 0.0
        self.velocity_publisher.publish(self.robot_velocity)

    def bug2_algorithm(self):
        # Implementa o algoritmo BUg 2
        while not rospy.is_shutdown():
            if not self.is_obstacle_detected:
                # Se não há obstáculos, mova-se em linha reta
                self.move_forward(1.0)
            else:
                # Se há obstáculo, rode em torno dele
                self.turn(1.57)  # Gire 90 graus (1.57 radianos)

                # Siga o contorno do obstáculo
                while self.is_obstacle_detected:
                    self.move_forward(0.5)

                # Continue em linha reta até que a distância ao obstáculo seja alcançada novamente
