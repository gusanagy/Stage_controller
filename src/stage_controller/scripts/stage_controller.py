#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import time
import random
from math import *
import tf

laser = LaserScan()
odometry = Odometry()

def odometry_callback(data):
    global odometry
    odometry = data

def laser_callback(data):
    global laser
    laser = data
class STage_Head():
	"""
	Esta Classe Funciona como esqueleto central deste script. Definindo o Goal a ser alcançado e 
	os nodos a seres iniciados no Ros.
	"""
	def __init__(self):
		# Inicializa o nó do ROS
		rospy.init_node("stage_controller_node", anonymous=False) 
		#Mensagens do Ros
		self.twist_msg = Twist()
		self.data_odom = Odometry()
		self.data_laser = LaserScan()

		# Adquire a posição absoluta no mapa
		self.pose_ground_truth = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.odometry_callback)
		#Adquire dados do sensor laser atraves de um subscriber
		self.laser_scan = rospy.Subscriber("/base_scan", LaserScan, self.laser_callback)
		# Cria um publisher para enviar comandos de velocidade para o robô
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		#-----------
		#Ponto De goal Objetivo final
		self.f_goal = Point()
		self.f_goal.x = 8.0
		self.f_goal.y = 3.0
		self.f_min_distance = 0.3
		self.goal_end = False
		#Ponto De goal Objetivo minimo
		self.goal = Point()
		self.goal.x = 0.0
		self.goal.y = 0.0
		self.min_distance = 0.3
		self.goal_end = False
	

		# Define uma taxa de atualização para a movimentação
		self.rate = rospy.Rate(10)
	def laser_callback(self,data_laser:LaserScan):

		self.data_laser = data_laser

	def odometry_callback(self,data_odom:Odometry):

		self.data_odom = data_odom
		
	def quaternium_angle(self):	
		orientacao_quaternion = self.data_odom.pose.pose.orientation

		# Converter a orientação do quaterniônio para Euler
		orientacao_euler = tf.transformations.euler_from_quaternion([
		orientacao_quaternion.x,
		orientacao_quaternion.y,
		orientacao_quaternion.z,
		orientacao_quaternion.w])

		return orientacao_euler[2]
	
	def GoTo_goal(self,goal_x,goal_y):
		self.goal.x = goal_x
		self.goal.y = goal_y
		#print(f"Minimum points {self.goal.x, self.goal.y}")
	
	def fix_angle(self, ang):
		# Normaliza o ângulo para o intervalo [-pi, pi]
		normalized_angle = (ang + pi) % (2 * pi) - pi
		return normalized_angle
	


	def GoTo(self):
		
		# Obtém a posição atual do robô
		current_position = self.data_odom.pose.pose.position

		# Calcula a diferença entre a posição atual e o objetivo
		dx = self.goal.x - current_position.x
		dy = self.goal.y - current_position.y

		# Calcula o ângulo em relação ao objetivo
		target_angle = self.fix_angle(atan2(dy, dx))
		current_angle = self.fix_angle(self.quaternium_angle())

		# Calcula a distância até o objetivo
		distance = sqrt(pow(dx,2) + pow(dy,2)) #distancia euclideana

		#arctan2(cos/sqrt(cos2 + sin2 ) , sin/sqrt(cos2 + sin2 ))

		ang = self.fix_angle(target_angle - current_angle)
		#ang = target_angle - current_angle
		
		if (len(self.data_laser.ranges) > 0):
				if (min(self.data_laser.ranges[45*4:255*4]) < random.uniform(0.09,0.2)):
					self.twist_msg.linear.x = -0.5
					self.twist_msg.angular.z = random.uniform(-2,2)
					self.cmd_vel_pub.publish(self.twist_msg)
					rospy.loginfo("Navegando...")
				elif (distance < random.uniform(0.1,0.5)):
					self.twist_msg.linear.x = 0.08  # Velocidade linear desejada
					self.twist_msg.angular.z = 0.1 * ang  # Velocidade angular desejada	
					self.cmd_vel_pub.publish(self.twist_msg)
				else:
					# Define a velocidade linear e angular para alcançar o objetivo
					# Publica os comandos de velocidade para mover o robô
					self.twist_msg.linear.x = 0.08  # Velocidade linear desejada
					self.twist_msg.angular.z = 0.1 * random.uniform(-ang/random.uniform(1,2),ang)  # Velocidade angular desejada	
					self.cmd_vel_pub.publish(self.twist_msg)	
			
		if distance < self.f_min_distance :
			# Para o movimento do robô
			self.twist_msg.linear.x = 0.0
			self.twist_msg.angular.z = 0.0
			self.cmd_vel_pub.publish(self.twist_msg)
			rospy.loginfo("Goal reached!")
			rospy.signal_shutdown("Rospy foi finalizado pois alcançou o goal")

		print(f"""
		GoTo ------------------
		distancia : {distance},
		Objetivo: {self.goal.x,self.goal.y},
		dx: {dx} ,dy: {dy},
		angulo target: {target_angle},
		angulo atual: {current_angle},
		twist ang: {self.twist_msg.angular.z}
		devo parar{distance < self.f_min_distance }
		
		""") 
		
		
if __name__ == "__main__":

	stage_robot = STage_Head()
	#stage_robot.GoTo_goal(goal_x = 3.0, goal_y = 7.0)
	#stage_robot.GoTo_goal(goal_x = 1.0, goal_y = 3.0)
	stage_robot.GoTo_goal(goal_x = -1.0, goal_y = 5.0)
	#stage_robot.GoTo_goal(goal_x = -1.0, goal_y = 4.0)
	while not rospy.is_shutdown():
		stage_robot.GoTo()
	#r.sleep()