#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image, Joy # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy
import math
import speech_recognition as sr #Importar Libreria Speech Recognition
from duckietown_msgs.msg import Twist2DStamped

"""
v.1.0.R
Este programa permite mover al Duckiebot dentro del simulador
usando control por voz.
Version basica, adaptado para ROS
"""

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
	
		# Drift
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
		
	def callback_voz(self, msg):
		texto = msg.data
		
		if texto == "avanzar":
			msg_rueda = Twist2DStamped()
			msg_rueda.v = 1
			msg_rueda.omega = 0
			
			self.pub_control.publish(msg_rueda)

def main():
	rospy.init_node("bot") #creacion y registro del nodo!
	
	# rosrun desafios_2022 Robot_interactivo.py 	
	obj = Template('args')
	
	rospy.spin()


if __name__ =='__main__':
	main()
