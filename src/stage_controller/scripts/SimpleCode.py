#! /usr/bin/env python3
import rospy 
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from math import *
import tf
import numpy as np
import random


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
		self.f_min_distance = 0.1
		self.goal_end = False
		#Ponto De goal Objetivo minimo
		self.goal = Point()
		self.goal.x = 0.0
		self.goal.y = 0.0
		self.min_distance = 0.1
		self.goal_end = False
		
		#state
		#self.states = ["goal_reached","wall_following","obstacle_avoidance"]
		self.state = "wait"
		


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
	def laser_data(self):
		print(f"""
			Laser_data -----------------
			angle_min: {self.data_laser.angle_min}
			angle_max: {self.data_laser.angle_max}
			angle_increment: {self.data_laser.angle_increment}
			range_min: {self.data_laser.range_min}
			range_max: {self.data_laser.range_max}
			range sel: {self.data_laser.ranges[ceil(len(self.data_laser.ranges)/2)-1:ceil(len(self.data_laser.ranges)/2)]}
			""")
		
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
		
		# print(f"""
		# 	GoTo ------------------
		# 	distancia : {distance},
		# 	Objetivo: {self.goal.x,self.goal.y},
		# 	dx: {dx} ,dy: {dy},
		# 	angulo target: {target_angle},
		# 	angulo atual: {current_angle},
		# 	twist ang: {self.twist_msg.angular.z}
			
		# #  """)

		

		obs_ang_r, obs_ang_l, obs_ang_c, range_left, range_right, range_center = self.Obstacle()

		if range_center < self.f_min_distance + 0.2:
			#print("")
			self.wall_following(current_ang = current_angle)
		else: 
			"""if distance < self.f_min_distance:
				# Para o movimento do robô
				self.twist_msg.linear.x = 0.0
				self.twist_msg.angular.z = 0.0
				self.cmd_vel_pub.publish(self.twist_msg)
				rospy.loginfo("Minimum position reached!")
				#rospy.signal_shutdown("Rospy foi finalizado pois alcançou o minimo")
			else:
				# Define a velocidade linear e angular para alcançar o objetivo
				# Publica os comandos de velocidade para mover o robô
				self.cmd_vel_pub.publish(self.twist_msg)
				self.twist_msg.linear.x = 0.2   # Velocidade linear desejada
				self.twist_msg.angular.z = 0.2 * ang  # Velocidade angular desejada	"""
			pass	
			
	def wall_following(self,current_ang):
		print("Entrou no wall_fall")
		obs_ang_r, obs_ang_l, obs_ang_c, range_left, range_right, range_center = self.Obstacle()
		while range_center < self.f_min_distance :
			self.twist_msg.linear.x = -0.2
			self.cmd_vel_pub.publish(self.twist_msg)

		if range_left > range_right:

			while self.fix_angle(obs_ang_l) != current_ang:
				self.twist_msg.linear.x = 0.0
				self.twist_msg.angular.z = 0.2 * self.fix_angle(obs_ang_l)
				self.cmd_vel_pub.publish(self.twist_msg)
		else:
			while self.fix_angle(obs_ang_r) != current_ang:
				self.twist_msg.linear.x = 0.0
				self.twist_msg.angular.z = 0.2 * self.fix_angle(obs_ang_r)
				self.cmd_vel_pub.publish(self.twist_msg)
		while obs_ang_c != current_ang: 
			self.twist_msg.linear.x = 0.3
			self.twist_msg.angular.z = 0.2
			self.cmd_vel_pub.publish(self.twist_msg)
			if range_center < self.f_min_distance:
				self.twist_msg.linear.x = 0.0
				self.twist_msg.angular.z = 0.0
				self.cmd_vel_pub.publish(self.twist_msg)
				break
		 

	def Obstacle(self):

		quarto = ceil(len(self.data_laser.ranges)/6)
		metade = ceil(len(self.data_laser.ranges)/2)

		range_left = 0.0
		for x in self.data_laser.ranges[quarto-1:quarto]:
			range_left = float(x)
		range_center = 0.0
		for x in self.data_laser.ranges[metade-1:metade]:
			range_center = float(x)
		range_right = 0.0
		for x in self.data_laser.ranges[(quarto*5)-1:quarto*5]:
			range_right = float(x)
			
		
		#obstaculo posições esquerda
		obst_pos_xl = cos(self.fix_angle(self.quaternium_angle())) * range_left
		obst_pos_yl = sin(self.fix_angle(self.quaternium_angle())) * range_left
		#obstaculo posições centro
		obst_pos_xc = cos(self.fix_angle(self.quaternium_angle())) * range_center
		obst_pos_yc = sin(self.fix_angle(self.quaternium_angle())) * range_center
		#obstaculo posições direita
		obst_pos_xr = cos(self.fix_angle(self.quaternium_angle())) * range_right
		obst_pos_yr = sin(self.fix_angle(self.quaternium_angle())) * range_right
		
		
		# Obtém a posição atual do robô
		current_position = self.data_odom.pose.pose.position

		# Calcula a diferença entre a posição atual e o obstaculo da esquerda
		dxl = obst_pos_xl - current_position.x
		dyl = obst_pos_yl- current_position.y
		# Calcula a diferença entre a posição atual e o obstaculo da direita
		dxr = obst_pos_xr - current_position.x
		dyr = obst_pos_yr- current_position.y
		# Calcula a difrença entre a posição atual e o obstaculo do centro
		dxc = obst_pos_xc - current_position.x
		dyc = obst_pos_yc- current_position.y

		obs_ang_r = atan2(cos(dxr)/sqrt(pow(cos(dxr),2) + pow(sin(dyr),2) ) , sin(dyr)/sqrt(pow(cos(dxr),2) + pow(sin(dyr),2) ))
		obs_ang_l = atan2(cos(dxl)/sqrt(pow(cos(dxl),2) + pow(sin(dyl),2) ) , sin(dyl)/sqrt(pow(cos(dxl),2) + pow(sin(dyl),2) ))
		obs_ang_c = atan2(cos(dxc)/sqrt(pow(cos(dxc),2) + pow(sin(dyc),2) ) , sin(dyc)/sqrt(pow(cos(dxc),2) + pow(sin(dyc),2) ))
		
		print(f"""
			esquera laser: {range_left},
			direita laser: {range_right},
			frente laser: {range_center}
			obstaculo direita: {obst_pos_xr,obst_pos_yr}
			obstaculo esquerda {obst_pos_xl,obst_pos_yl}
			obstaculo frente:  {obst_pos_xc,obst_pos_yc}
			ang right: {obs_ang_r}
			ang left: {obs_ang_l}
			ang frente: {obs_ang_c}
			
		""")

		return obs_ang_r, obs_ang_l, obs_ang_c, range_left, range_right, range_center


	def field(self):
		pass
		

	def Goal(self):
		#print(laser.ranges)
		
		x = self.data_odom.pose.pose.position.x
		y = self.data_odom.pose.pose.position.y

		# Verifica se chegou ao alvo
		distance = sqrt((x-self.f_goal.x)**2 + (y-self.f_goal.y)**2)

		if self.n_goal is True:
			self.goal_end = False
			
		if (distance > self.min_distance):
			#rospy.loginfo("Where i am: X: %s, Y: %s", x, y)
			# FACA O SEU CODIGO AQUI
			pass
		else:
			self.goal_end = True
			#rospy.signal_shutdown("Rospy foi finalizado pois alcançou o minimo")
			rospy.loginfo("Alvo Alcancado!!!!!")	

if __name__ == "__main__": 

	#Inicia classe, nodos (feito)
	#Classe de posicionamento do robo (feito)
	stage_robot = STage_Head()
	#Classe de GoTo (feito)

	#Classe para A*

	#Classe para o Tensor Fields

	#Movimentar usando GoTo como saida do A*
	stage_robot.GoTo_goal(goal_x = 3.0, goal_y = 8.0)
	while not rospy.is_shutdown():
        # Define a posição de destino
		# Cria uma instância do objeto GotoPosition e inicia o movimento
		#stage_robot.laser_data()
		stage_robot.GoTo()
	


