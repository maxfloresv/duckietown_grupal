#!/usr/bin/env python

# ROS cosas:
import rospy 
# http://wiki.ros.org/std_msgs
from std_msgs.msg import String, Int32, Empty
# http://wiki.ros.org/geometry_msgs
from geometry_msgs.msg import Twist
# http://wiki.ros.org/sensor_msgs
from sensor_msgs.msg import Image, Joy
# Se pueden ver dentro del Duckiebot
from duckietown_msgs.msg import Twist2DStamped

import math
import numpy as np
from time import time, sleep
from random import randint

import pyttsx3
eng = pyttsx3.init()

# Propiedades de voz
eng.setProperty('voice', 'spanish-latin-am')
eng.setProperty('volume', 1.0)

eng.say("Quack quack!!!!")
#eng.runAndWait()

class Template(object):
	# Calcula el tiempo que le toma al duckiebot girar en un angulo
	def tiempo(self, angulo):
		# Calculado a partir de angulo = vel. angular * tiempo
		t_vuelta = 1.5 # Vuelta de 2pi
		# Velocidad angular
		omega_real = (2 * math.pi) / t_vuelta
		# theta = omega*t
		return angulo / omega_real
		
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		
		# Subscribers:
		rospy.Subscriber("/duckiebot/joy", Joy, self.callback_control)
		rospy.Subscriber("/duckiebot/voz/v2t", String, self.callback_voz)
		rospy.Subscriber("/duckiebot/voz/tiempo", String, self.callback_tiempo)
		rospy.Subscriber("/duckiebot/voz/wiki", String, self.callback_wiki)
		
		# Publishers:
		self.pub_control = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size=10)
		self.pub_voz = rospy.Publisher("/duckiebot/voz/publicar_msg", String, queue_size=10)
		
		# Extras:
		self.instrucciones = []
		self.vel_lineal = 0
		self.vel_angular = 0
		self.t_recuperado = 0
		self.valid = False
		self.inst_inversa = {
			"avanzar": "retroceder",
			"retroceder": "avanzar",
			"izquierda": "derecha",
			"derecha": "izquierda",
			"girar": "girar_d" # voltear_d = voltear inversa
		}
		self.propiedades = {
			"avanzar": [-5, 0, 3],
			"retroceder": [5, 0, 3],
			"izquierda": [0, 10, self.tiempo(math.pi / 2)],
			"derecha": [0, -10, self.tiempo(math.pi / 2)],
			"girar": [0, 10, self.tiempo(math.pi)],
			"girar_d": [0, -10, self.tiempo(math.pi)]
		}
		
		self.chistes = ["como termina una carrera entre dos patos... empatados", 
		"que pasa cuando tiras un pato a una piscina... nada", 
		"que hace un pato en una pista de hielo... patina", 
		"como se aman los patos... pa toda la vida"]
		
		self.canciones = [ "dale a tu cuerpo alegria macarena",
		"torero, poner el alma en el ruedo, no importa lo que se venga pa que sepas que te quiero",
		"ama barbi gerl, in a barbi weeerld, laif in plastic is fantastic"]
		
	def callback_control(self, msg):
		axes = list(msg.axes)
		buttons = list(msg.buttons)
	
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
		
		B = buttons[1]
		
		# Freno de emergencia
		if B == 1:
			mensaje.v = 0
			mensaje.omega = 0

		self.pub_control.publish(mensaje)
		
	# Calcula la distancia entre dos strings (ADAPTADO A Python 2.7)
	# https://stackabuse.com/levenshtein-distance-and-text-similarity-in-python/
	def levenshtein(self, seq1, seq2):
		size_x = len(seq1) + 1
		size_y = len(seq2) + 1
		matrix = np.zeros((size_x, size_y))
			
		for x in range(size_x):
			matrix[x][0] = x

		for y in range(size_y):
			matrix[0][y] = y
				
		for x in range(1, size_x):
			for y in range(1, size_y):
				if seq1[x-1] == seq2[y-1]:
					matrix[x][y] = min(
						matrix[x-1][y] + 1, 
						matrix[x-1][y-1], 
						matrix[x][y-1] + 1
					)
				else:
					matrix[x][y] = min(
						matrix[x-1][y] + 1, 
						matrix[x-1][y-1] + 1, 
						matrix[x][y-1] + 1
					)
			
		return int(matrix[size_x - 1][size_y - 1])
	
	# Ejecuta una instruccion que se haya pedido, devolviendo False
	# si NO coincide y True si es la indicada
	def ejecutar_instruccion(self, texto, instruccion, v_lin, v_ang, t, inversa=False):
		# Maxima diferencia entre strings para considerar la instruccion
		MAX_DIST = 2

		distancia = self.levenshtein(texto, instruccion)

		if distancia <= MAX_DIST:
			if instruccion == "avanzar" or instruccion == "retroceder":
				eng.say("Durante cuantos segundos?")
				eng.runAndWait()
				
				self.pub_voz.publish("tiempo")
				sleep(5)
				
				while self.valid == False:
					eng.say("Por favor, di un numero entero positivo")
					eng.runAndWait()
					self.pub_voz.publish("tiempo")
					sleep(5)
				
				t = self.t_recuperado
		
			eng.say(instruccion)
			eng.runAndWait()
			
			msg_rueda = Twist2DStamped()
			
			# No queremos agregar instrucciones cuando lo llamamos desde volver	
			if not inversa:
				self.instrucciones.append(self.inst_inversa[instruccion])

			self.v_lineal = v_lin
			self.v_angular = v_ang

			t_actual = time()
			
			msg_rueda.v = self.v_lineal
			msg_rueda.omega = self.v_angular
			
			# Ejecutar durante t segundos
			while time() - t_actual <= t:
				self.pub_control.publish(msg_rueda)

			# Pasados los t segundos, que frene absolutamente
			msg_rueda.v = 0
			msg_rueda.omega = 0
			
			self.pub_control.publish(msg_rueda)
			
			# Reseteamos el tiempo recuperado
			self.t_recuperado = 0
			self.valid = False

			return True

		return False
		
	def callback_voz(self, msg):
		texto = msg.data
		
		instrucciones = ["avanzar", "retroceder", "izquierda", "derecha", "girar"]
		
		# Propiedades de cada accion 
		# Tiene la forma [vel_lineal, vel_angular, tiempo]
		propiedades = self.propiedades
		
		for inst in instrucciones:
			prop_inst = propiedades[inst]
			
			# Extraemos las velocidades y el tiempo de ejecucion
			v_lin, v_ang, tiempo = prop_inst
			
			# Si la instruccion se ejecuta, paramos (porque era la indicada)
			if self.ejecutar_instruccion(texto, inst, v_lin, v_ang, tiempo):
				return
				
		MAX_DIST = 2
		bailar_dist = self.levenshtein(texto, "bailar")
		if bailar_dist <= MAX_DIST:
			cancion = randint(0, len(self.canciones)-1)
			decir(self.canciones[cancion])
			
			msg_rueda = Twist2DStamped()
			
			instrucciones = {
				1: [0, -10, self.tiempo(math.pi / 4)],
				2: [0, 10, self.tiempo(math.pi / 2)],
				3: [0, -10, self.tiempo(math.pi / 2)],
				4: [0, 10, self.tiempo(math.pi / 2)],
				5: [0, -10, self.tiempo(math.pi / 4)],
				6: [8, 0, 0.25],
				7: [-8, 0, 0.5],
				8: [8, 0, 0.5],
				9: [-8, 0, 0.25]				
			}
			
			for i in range(1, 10):
				v_lin, v_ang, t = instrucciones[i]
					
				msg_rueda.omega = v_ang
				msg_rueda.v = v_lin
				
				t_actual = time()

				# Ejecutar durante t segundos
				while time() - t_actual <= t:
					self.pub_control.publish(msg_rueda)
				
				msg_rueda.v = 0
				msg_rueda.omega = 0	
				
				self.pub_control.publish(msg_rueda)
			
			return
	
		volver_dist = self.levenshtein(texto, "volver")
		vuelve_dist = self.levenshtein(texto, "vuelve")
		
		dist = min(volver_dist, vuelve_dist)
		
		if dist <= MAX_DIST:
			for i in range(len(self.instrucciones)):
				largo = len(self.instrucciones)

				inst = self.instrucciones[largo - 1 - i]
				
				v_lin, v_ang, t = self.propiedades[inst]
				
				self.ejecutar_instruccion(inst, inst, v_lin, v_ang, t, True)
				
			self.instrucciones = []
			return
		
		borrar_dist = self.levenshtein(texto, "borrar")
		if borrar_dist <= MAX_DIST:
			self.instrucciones = []
			
		pista_dist = self.levenshtein(texto, "pista")
		if pista_dist <= MAX_DIST:
			msg_rueda = Twist2DStamped()
			
			I_LIM = 8
			
			instrucciones = {
				1: [-10, 0, 3], 
				2: [0, 10, self.tiempo(math.pi / 3)],
				3: [-10, 0, 1.4],
				4: [0, 10, self.tiempo(math.pi / 4)],
				5: [-10, 0, 2],
				6: [0, 10, self.tiempo(math.pi / 4.2)],
				7: [-10, 0, 1.7],
				8: [0, 10, self.tiempo(math.pi / 3)],

			}
			
			for i in range(1, I_LIM+1):
				v_lin, v_ang, t = instrucciones[i]
					
				msg_rueda.omega = v_ang
				msg_rueda.v = v_lin
				
				t_actual = time()

				# Ejecutar durante t segundos
				while time() - t_actual <= t:
					self.pub_control.publish(msg_rueda)
				
				msg_rueda.v = 0
				msg_rueda.omega = 0	
				
				self.pub_control.publish(msg_rueda)
			
			return

		buscar_dist = self.levenshtein(texto, "buscar")
		if buscar_dist <= MAX_DIST:
			self.pub_voz.publish("buscar")
			return
			
		chiste_dist = self.levenshtein(texto, "chiste")
		if chiste_dist <= MAX_DIST:
			chiste = randint(0, len(self.chistes)-1)
			decir(self.chistes[chiste])


	def callback_tiempo(self, msg):
		texto = msg.data
		# Por algun motivo, el uno no lo toma como entero
		if texto == "uno":
			self.valid = True
			self.t_recuperado = 1
		elif not texto.isdigit():
			self.valid = False
		else:
			self.valid = True
			self.t_recuperado = int(msg.data)
			
	def callback_wiki(self, msg):
		texto = msg.data
		eng.say(texto)
		eng.runAndWait()

def main():
	# Nodo local del Duckiebot
	rospy.init_node("bot") 
	
	print("Funcionando!")
	
	obj = Template('args')
	
	# Loop
	rospy.spin()

if __name__ == '__main__':
	main()
