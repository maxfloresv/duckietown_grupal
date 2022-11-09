#!/usr/bin/env python

# ROS cosas:
import rospy 
# http://wiki.ros.org/std_msgs
from std_msgs.msg import String, Int32 
# http://wiki.ros.org/geometry_msgs
from geometry_msgs.msg import Twist
# http://wiki.ros.org/sensor_msgs
from sensor_msgs.msg import Image, Joy
# Se pueden ver dentro del Duckiebot
from duckietown_msgs.msg import Twist2DStamped

# Procesamiento de imgs con ML:
import cv2 
from cv_bridge import CvBridge

import math
import numpy as np
from time import time

# To-do: pip install polyleven en el duckiebot!!
from polyleven import levenshtein

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		
		# Subscribers:
		rospy.Subscriber("/duckiebot/camera_node/image/raw", Image, self.callback_camara)
		rospy.Subscriber("/duckiebot/joy", Joy, self.callback_control)
		rospy.Subscriber("/duckiebot/voz/v2t", String, self.callback_voz)
		
		# Publishers:
		self.pub_camara = rospy.Publisher("/duckiebot/camera_node/image/test", Image, queue_size=1)
		self.pub_control = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped)
		
		# Extras:
		self.instrucciones = []
		self.vel_lineal = 0
		self.vel_angular = 0
		

	def callback_camara(self, msg):
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(msg, "bgr8")
		
		image_out = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
		lower_limit = np.array([25, 130, 130])
		upper_limit = np.array([35, 255, 255])

		mask = cv2.inRange(image_out, lower_limit, upper_limit)

		# Refinando la imagen
		kernel = np.ones((3, 3), np.uint8)
		img_out = cv2.erode(mask, kernel, iterations=1)
		img_out = cv2.dilate(img_out, kernel, iterations=3)

		img_masked = cv2.bitwise_and(image, image, mask=img_out)

		# Creando el contorno
		_, contours, hierarchy = cv2.findContours(img_out, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		min_area = 5

		for val in contours:
			x, y, w, h = cv2.boundingRect(val)
			if w*h > min_area:
				cv2.rectangle(image, (x+w, y+h), (x,y), (0, 0, 0), 2)
				d_r = 3.5
				f = 101.8
				p = h
				
				D_r = (d_r * f)/p

				min_dist = 10
				max_dist = 50
				
				if D_r <= min_dist or D_r > max_dist:
					msg_rueda = Twist2DStamped()
					msg_rueda.v = 0
					msg_rueda.omega *= 10
					
					self.pub_control.publish(msg_rueda)
		
		msg = bridge.cv2_to_imgmsg(image, "bgr8")

		self.pub_camara.publish(msg)

	def callback_control(self, msg):
		axes = list(msg.axes)
		buttons = list(msg.buttons)

		B = buttons[1]
		
		# Freno de emergencia
		if B == 1:
			for i in range(len(axes)):
				axes[i] = 0
	
		# Control del drift
		drift_tol = 0.1
		if abs(axes[0]) <= drift_tol:
			axes[0] = 0

		axes[0] *= (4 * math.pi)

		mensaje = Twist2DStamped()

		# Freno por si acelera y retrocede
		if axes[2] == -1 and axes[5] == -1:
			mensaje.v = 0

		# Retroceso
		elif axes[2] == -1:
			axes[2] *= -1
			mensaje.v = axes[2]
		
		# Avance
		elif axes[5] == -1:
			mensaje.v = axes[5]

		mensaje.omega = axes[0]

		self.pub_control.publish(mensaje)
	
	# Calcula el tiempo que le toma al duckiebot girar en un angulo
	def tiempo(angulo):
		# Calculado a partir de angulo = vel. angular * tiempo
		t_vuelta = 1.2 # Vuelta de 2pi
		return 1000 * angulo / ((2 * math.pi) / t_vuelta)
	
	# Ejecuta una instruccion que se haya pedido, devolviendo False
	# si NO coincide y True si es la indicada
	def ejecutar_instruccion(texto, instruccion, v_lin, v_ang, t):
		# Maxima diferencia entre strings para considerar la instruccion
		MAX_DIST = 2

		distancia = levenshtein(texto, instruccion)

		if distancia <= MAX_DIST:
			msg_rueda = Twist2DStamped()
			# TBA: self.instrucciones.append(inst_inversa(L[0]))

			self.v_lineal = v_lin
			self.v_angular = v_ang

			t_actual = time()

			# Ejecutar durante t segundos
			while time() - t_actual <= t:
				msg_rueda.v = v_lin
				msg_rueda.omega = v_ang

			self.pub_control.publish(msg_rueda)

			return True

		return False
		
	def callback_voz(self, msg):
		texto = msg.data
		
		instrucciones = ["avanzar", "retroceder", "izquierda", "derecha", "frenar", "voltear"]
		
		# Propiedades de cada accion 
		# Tiene la forma [vel_lineal, vel_angular, tiempo]
		propiedades = {
			"avanzar": [-1, self.vel_angular, 5],
			"retroceder": [1, self.vel_angular, 5],
			"izquierda": [self.vel_lineal, 1, self.tiempo(math.pi / 4)],
			"derecha": [self.vel_lineal, -1, self.tiempo(math.pi / 4)],
			"frenar": [0, 0, 0],
			"voltear": [self.vel_lineal, 1, self.tiempo(math.pi)]
		}
		
		for inst in instrucciones:
			prop_inst = propiedades[inst]
			
			# Extraemos las velocidades y el tiempo de ejecucion
			v_lin, v_ang, tiempo = prop_inst
			
			# Si la instruccion se ejecuta, paramos (porque era la indicada)
			if self.ejecutar_instruccion(texto, inst, v_lin, v_ang, tiempo):
				return
	
		# TO-DO: Volver, y baile [antes del 15/11]

def main():
	# Nodo local del Duckiebot
	rospy.init_node("bot") 
	
	obj = Template('args')
	
	# Loop
	rospy.spin()


if __name__ == '__main__':
	main()
