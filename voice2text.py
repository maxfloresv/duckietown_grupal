
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import cv2
import time

import speech_recognition as sr

global ejecutar
ejecutar = True

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		
		# Publisher de voz
		# latch=True para publicar mensajes perdidos
		# https://stackoverflow.com/a/60661094
		self.pub = rospy.Publisher("/duckiebot/voz/v2t", String, queue_size=1, latch=True)
		
		rospy.Subscriber("/duckiebot/joy", Joy, self.callback)

		# Programa de voz
		self.r = sr.Recognizer()

	def callback(self, msg):
		buttons = list(msg.buttons)
		
		A = buttons[0]

		# Ejecutar el microfono
		if A == 1:
			with sr.Microphone() as source:
				print("Quack quack...") # que lo diga
				audio = self.r.listen(source, None, 1.5)
				
				try:
					text = self.r.recognize_google(audio, language='es-ES')
					print("Lo quack dijiste fue:", str(text))
					msg = String()
					msg.data = str(text)
					self.pub.publish(msg)
					
				except Exception as e:
					print("No quackche", str(e))
		
def main():
	# Nodo local del PC
	rospy.init_node("pc")
	
	print("Funcionando!")
	
	obj = Template('args')
		
	rospy.spin()
	# Ejecutar programa cada 10ms
	"""
	rate = rospy.Rate(10)
	while (not rospy.is_shutdown()) and ejecutar:
		print(True)
		obj.callback()
		rate.sleep()"""

if __name__ == '__main__':
	main()
