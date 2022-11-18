import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import cv2
import time
import speech_recognition as sr

# Mientras run=True, corre el programa
run = True
active = False

class Template(object):
    def __init__(self, args):
        super(Template, self).__init__()
        self.args = args

        # Publisher de voz
        # latch=True para publicar mensajes perdidos
        # https://stackoverflow.com/a/60661094
        self.pub = rospy.Publisher("/duckiebot/voz/v2t", String, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber("/duckiebot/joy", Joy, self.callback)

        # Programa de voz
        self.r = sr.Recognizer()

    def callback(self, msg):
        # Modifico los valores globales
        global run, active
        buttons = list(msg.buttons)

        A = buttons[0]
        X = buttons[2]
        
        if X == 1:
            run = False
            return

        # Ejecutar el microfono solo si no esta activo (evita lag)
        if A == 1 and not active:
            active = True
            with sr.Microphone() as source:
                print("Quack quack...")  # que lo diga
                audio = self.r.listen(source, None, 1.5)

                try:
                    text = self.r.recognize_google(audio, language='es-ES')
                    print("Lo quack dijiste fue:", str(text))
                    msg = String()
                    msg.data = str(text)
                    self.pub.publish(msg)

                except Exception as e:
                    print("No quackche", str(e))
                    
                active = False


def main():
    # Nodo local del PC
    rospy.init_node("pc")

    print("Funcionando!")

    obj = Template('args')

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if run == False:
            sys.exit()
            
        obj.callback()
        rate.sleep()

if __name__ == '__main__':
    main()
