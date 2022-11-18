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

# Freno de emergencia por voz
freno = False

class Template(object):
    # Calcula el tiempo que le toma al duckiebot girar en un angulo
    def tiempo(self, angulo):
        # Calculado a partir de angulo = vel. angular * tiempo
        t_vuelta = 1.2  # Vuelta de 2pi
        return angulo / ((2 * math.pi) / t_vuelta)

    def __init__(self, args):
        super(Template, self).__init__()
        self.args = args

        # Subscribers:
        rospy.Subscriber("/duckiebot/camera_node/image/raw", Image, self.callback_camara)
        rospy.Subscriber("/duckiebot/joy", Joy, self.callback_control)
        rospy.Subscriber("/duckiebot/voz/v2t", String, self.callback_voz)

        # Publishers:
        self.pub_camara = rospy.Publisher("/duckiebot/camera_node/image/test", Image, queue_size=1)
        self.pub_control = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size=1)

        # Extras:
        self.instrucciones = []
        self.vel_lineal = 0
        self.vel_angular = 0
        self.inst_inversa = {
            "avanzar": "retroceder",
            "retroceder": "avanzar",
            "izquierda": "derecha",
            "derecha": "izquierda",
            "voltear": "voltear_d"  # voltear_d = voltear inversa
        }
        self.propiedades = {
            "avanzar": [-5, self.vel_angular, 15],
            "retroceder": [5, self.vel_angular, 5],
            "izquierda": [self.vel_lineal, 10, self.tiempo(math.pi / 2)],
            "derecha": [self.vel_lineal, -10, self.tiempo(math.pi / 2)],
            "frenar": [-532, 0, -1],
            "voltear": [self.vel_lineal, 10, self.tiempo(math.pi)],
            "voltear_d": [self.vel_lineal, -10, self.tiempo(math.pi)]
        }

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
                if seq1[x - 1] == seq2[y - 1]:
                    matrix[x][y] = min(
                        matrix[x - 1][y] + 1,
                        matrix[x - 1][y - 1],
                        matrix[x][y - 1] + 1
                    )
                else:
                    matrix[x][y] = min(
                        matrix[x - 1][y] + 1,
                        matrix[x - 1][y - 1] + 1,
                        matrix[x][y - 1] + 1
                    )

        return int(matrix[size_x - 1][size_y - 1])

    # Ejecuta una instruccion que se haya pedido, devolviendo False
    # si NO coincide y True si es la indicada
    def ejecutar_instruccion(self, texto, instruccion, v_lin, v_ang, t):
        global freno
        
        # Maxima diferencia entre strings para considerar la instruccion
        MAX_DIST = 2

        distancia = self.levenshtein(texto, instruccion)

        if distancia <= MAX_DIST:
            msg_rueda = Twist2DStamped()
            freno = False

            # Frena
            if v_lin == -532:
                freno = True

            else:
                self.instrucciones.append(self.inst_inversa[instruccion])

            self.v_lineal = v_lin
            self.v_angular = v_ang

            msg_rueda.v = v_lin
            msg_rueda.omega = v_ang

            t_actual = time()

            # Ejecutar durante t segundos
            while time() - t_actual <= t:
                if freno == True:
                    print("frenafrena")
                    msg_rueda.v = 0
                    msg_rueda.omega = 0

                    self.v_lineal = 0
                    self.v_angular = 0

                    self.pub_control.publish(msg_rueda)
                    break
                else:
                    self.pub_control.publish(msg_rueda)

            # Pasados los t segundos, que frene absolutamente
            msg_rueda.v = 0
            msg_rueda.omega = 0

            self.pub_control.publish(msg_rueda)

            return True

        return False

    def callback_voz(self, msg):
        global freno
        texto = msg.data

        instrucciones = ["avanzar", "retroceder", "izquierda", "derecha", "frenar", "voltear"]

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
            msg_rueda = Twist2DStamped()

            instrucciones = {
                1: [0, -10, self.tiempo(math.pi / 4)],
                2: [0, 10, self.tiempo(math.pi / 2)],
                3: [0, -10, self.tiempo(math.pi / 2)],
                4: [0, 10, self.tiempo(math.pi / 2)],
                5: [0, -10, self.tiempo(math.pi / 4)],
                6: [10, 0, 0.5],
                7: [-10, 0, 1],
                8: [10, 0, 1],
                9: [-10, 0, 0.5]
            }

            for i in range(1, 10):
                v_lin, v_ang, t = instrucciones[i]

                msg_rueda.omega = v_ang
                msg_rueda.v = v_lin

                t_actual = time()

                # Ejecutar durante t segundos
                while time() - t_actual <= t:
                    if freno == True:
                        msg_rueda.v = 0
                        msg_rueda.omega = 0

                        self.v_lineal = 0
                        self.v_angular = 0

                        self.pub_control.publish(msg_rueda)
                        break
                    else:
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

                self.ejecutar_instruccion(inst, inst, v_lin, v_ang, t)

            return

        borrar_dist = self.levenshtein(texto, "borrar")
        if borrar_dist <= MAX_DIST:
            self.instrucciones = []

def main():
    # Nodo local del Duckiebot
    rospy.init_node("bot")

    obj = Template('args')

    # Loop
    rospy.spin()

if __name__ == '__main__':
    main()
