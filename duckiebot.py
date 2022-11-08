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
		
	# max_levenshtein: str list -> int
	# Dado un string s, calcula la minima distancia de Levenshtein
	# entre s y todos los elementos de la lista L.
	def min_levenshtein(s, L):
		dist = math.inf
		for i in range(L):
			dist = min(dist, levenshtein(s, L[i]))
				
		return dist
	
	# tiempo: int
	# Calcula el tiempo que le toma al duckiebot girar en un angulo
	def tiempo(angulo):
		# Calculado a partir de angulo = vel. angular * tiempo
		t_vuelta = 1.2 # Vuelta de 2pi
		return 1000 * angulo / ((2 * math.pi) / t_vuelta)
		
	def callback_voz(self, msg):
		texto = msg.data
		
		# Mensaje a publicar en las ruedas
		msg_rueda = Twist2DStamped()
		
		# Maxima diferencia entre strings para considerar la instruccion
		MAX_DIST = 2
		
		# Instrucciones inversas para volver
		inst_inversa = {
			"avanzar": "retroceder", 
			"retroceder": "avanzar", 
			"izquierda": "derecha", 
			"derecha": "izquierda",
			"voltear": "voltear"
		}
		
		# Avance:
		avanzar = ["avanzar", "acelerar", "adelante"]
		dist = self.min_levenshtein(texto, avanzar)
		
		if dist <= MAX_DIST:
			self.instrucciones.append(inst_inversa["avanzar"])
			
			msg_rueda.v = -1
			self.vel_lineal = -1
			
			msg_rueda.omega = 0
			self.vel_angular = 0
			
			self.pub_control.publish(msg_rueda)
			return
		
		# Retroceso:
		retroceder = ["retroceder", "atras"]
		dist = self.min_levenshtein(texto, retroceder)
		
		if dist <= MAX_DIST:
			self.instrucciones.append(inst_inversa["retroceder"])	
			
			msg_rueda.v = 1
			self.vel_lineal = 1
			
			msg_rueda.omega = 0
			self.vel_angular = 0
			
			self.pub_control.publish(msg_rueda)
			return
			
		# Frenado:
		frenar = ["frenar", "parar"]
		dist = self.min_levenshtein(texto, frenar)
		
		if dist <= MAX_DIST:		
			msg_rueda.v = 0
			self.vel_lineal = 0
			
			msg_rueda.omega = 0
			self.vel_angular = 0
			
			self.pub_control.publish(msg_rueda)
			return
		
		# Giro izq:
		izquierda = ["izquierda", "gira a la izquierda", "giro izquierda"]
		dist = self.min_levenshtein(texto, izquierda)
			
		if dist <= MAX_DIST:
			self.instrucciones.append(inst_inversa["izquierda"])	
						
			msg_rueda.v = self.vel_lineal
			
			t_actual = time() * 1000
			t_final = t_actual
			
			while t_final - t_actual <= self.tiempo(math.pi / 4):
				t_final = time() * 1000
				msg_rueda.omega = 1
			
			self.pub_control.publish(msg_rueda)	
			return
		
		# Giro derecha:
		derecha = ["derecha", "gira a la derecha", "gira derecha", "giro derecha"]
		dist = self.min_levenshtein(texto, derecha)
		
		if dist <= MAX_DIST:
			self.instrucciones.append(inst_inversa["derecha"])	
			
			msg_rueda.v = self.vel_lineal
			
			t_actual = time() * 1000
			t_final = t_actual
			
			while t_final - t_actual <= self.tiempo(math.pi / 4):
				t_final = time() * 1000
				msg_rueda.omega = -1 
			
			self.pub_control.publish(msg_rueda)
			return
		
		# Voltear:
		voltear = ["voltear", "cambiar el sentido", "cambiar sentido"]
		dist = self.min_levenshtein(texto, voltear)
		
		if dist <= MIN_DIST:
			self.instrucciones.append(inst_inversa["voltear"])	
						
			msg_rueda.v = self.vel_lineal
			
			t_actual = time() * 1000
			t_final = t_actual
			
			while t_final - t_actual <= self.tiempo(math.pi):
				t_final = time() * 1000
				msg_rueda.omega = 1 
			
			self.pub_control.publish(msg_rueda)
			return
				
		# Faltan: baila y vuelve

def main():
	# Nodo local del Duckiebot
	rospy.init_node("bot") 
	
	obj = Template('args')
	
	# Loop
	rospy.spin()


if __name__ == '__main__':
	main()
