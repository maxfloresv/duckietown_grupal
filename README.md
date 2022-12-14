# 🦆 Duckiebot interactivo
Este proyecto, desarrollado por [Nacho Dagach](https://github.com/ignaciodagachabugattas), [Pichi](https://github.com/pichiuwu) y [Max](https://github.com/maxfloresv), se está realizando para el curso **CD2201 Módulo Interdisciplinario** (Duckietown).

Se trata de un robot que escucha instrucciones y responde mediante voz. Algunas de las interacciones adicionales que se buscan implementar son:

* Rastrear instrucciones para volver a un cierto punto en el pasado.
* Hacer que el Duckiebot pueda decir datos curiosos/chistes.
* Hacer que el Duckiebot sea capaz de bailar con una instrucción.
* Agregar palabras clave ocultas de interacción para realizar acciones secretas (*Easter Eggs*).

La información sobre los avances del proyecto puede ser encontrada en la [wiki](https://github.com/maxfloresv/robot_interactivo/wiki).

### 🎮 Manejo del Joystick
Para el desarrollo del proyecto, se ocupó un control de Xbox 360, y se usaron los siguientes botones:
* Palanca izq. para el giro (3 en la imagen).
* LT para retroceder (6 en la imagen). 
* RT para acelerar (9 en la imagen).

La referencia al control está en la función `callback_control` del [archivo principal](https://github.com/maxfloresv/robot_interactivo/blob/main/duckiebot.py).

![d10437ca-81da-48db-a7a0-4c1173ad42e1](https://user-images.githubusercontent.com/45862114/199054372-978e232f-ea52-479f-8155-ffc665857241.png)

### 📚 Comandos y definiciones útiles:
El siguiente enlace [(click acá)](https://colab.research.google.com/drive/12NgKqmJJR6yABCUIHCZr6NJvG4wegARU) contiene una guía sobre el uso del Duckiebot.

### 👾¿Cómo lo utilizamos?
Para comenzar a utilizar el Duckiebot, se deben seguir lo siguiente. Primero se abre la consola en la cual se debe ingresar al Duckiebot ingresando `ssh -X duckiebot@duckiebot.local` y se ingresa la contraseña. Luego se abren las carpetas ingresando `cd \duckiebot\catkin_ws\src\desafios_2022\src`. Se ejecuta `roslaunch ros_cap duckie_core.local` y se ejecuta el programa.
Para esto se abre una nueva terminal en la consola y se utiliza `ifconfig` la cual se utiliza para configurar y ver el estado de las interfaces de red en los sistemas operativos Linux. Copiamos la IP encontrada y conectamos el duckiebot con el computador a travéz de `export ROS_IP=...` y pegamos justo a continuación la IP copiada anteriormente. Se ejecuta `cd duckiebot_pc` y luego `python3 voice2text.py` para ejecutar el programa del PC.

### 📄 Documentación adicional:
