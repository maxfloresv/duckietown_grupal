import rospy
from std_msgs.msg import String

import speech_recognition as sr

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		
		# Publisher de voz
		self.pub = rospy.Publisher("/duckiebot/voz/v2t", String, queue_size=1)

		# Programa de voz
		self.r = sr.Recognizer()

	def callback(self):
		with sr.Microphone() as source:
			print("Quack quack...") # que lo diga
			audio = self.r.listen(source, None, 3)
			
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
	
	obj = Template('args')
	
	# Ejecutar programa cada 10ms
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		obj.callback()
		rate.sleep()

if __name__ == '__main__':
	main()
