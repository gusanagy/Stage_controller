#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import atan2, pow, sqrt

class GotoPosition:
    def __init__(self, goal):
        # Inicializa o nó do ROS
        #rospy.init_node('goto_position_node', anonymous=True)

        # Define o objetivo de posição
        self.goal = goal

        # Cria um objeto do tipo Twist para controlar a velocidade do robô
        self.twist_msg = Twist()

        # Cria um publisher para enviar comandos de velocidade para o robô
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Cria um subscriber para obter a posição atual do robô
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Define uma taxa de atualização para a movimentação
        self.rate = rospy.Rate(10)

    def odom_callback(self, data):
        # Obtém a posição atual do robô
        current_position = data.pose.pose.position

        # Calcula a diferença entre a posição atual e o objetivo
        dx = self.goal.x - current_position.x
        dy = self.goal.y - current_position.y

        # Calcula o ângulo em relação ao objetivo
        target_angle = atan2(dy, dx)

        # Calcula a distância até o objetivo
        distance = sqrt(pow(dx, 2) + pow(dy, 2))

        # Define a velocidade linear e angular para alcançar o objetivo
        self.twist_msg.linear.x = 0.5  # Velocidade linear desejada
        self.twist_msg.angular.z = 0.5 * (target_angle - current_angle)  # Velocidade angular desejada

        # Publica os comandos de velocidade para mover o robô
        self.cmd_vel_pub.publish(self.twist_msg)

        # Verifica se o objetivo foi alcançado
        if distance < 0.1:
            # Para o movimento do robô
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist_msg)

            # Encerra o nó do ROS
            rospy.signal_shutdown('Reached the minimum goal')

    def goto_position(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        # Define a posição de destino
        goal = Point()
        goal.x = 1.0
        goal.y = 2.0

        # Cria uma instância do objeto GotoPosition e inicia o movimento
        goto = GotoPosition(goal)
        goto.goto_position()
    except rospy.ROSInterruptException:
        pass
