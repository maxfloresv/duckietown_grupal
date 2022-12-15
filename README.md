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
Para comenzar a utilizar el Duckiebot, se debe seguir lo siguiente. 

Primero se abre la consola en la cual se debe ingresar al Duckiebot utilizando `ssh -X duckiebot@duckiebot.local` y se ingresa la contraseña. Luego se abren las carpetas ingresando `cd \duckiebot\catkin_ws\src\desafios_2022\src`. Se ejecuta `roslaunch ros_cap duckie_core.local` y se ejecuta el programa en una nueva terminal desde el duckiebot con rosrun desafios_2022 duckiebot.py.

Para esto se abre una nueva terminal en la consola y se utiliza `ifconfig` la cual se usa para configurar y ver el estado de las interfaces de red en los sistemas operativos Linux. 

Luego copiamos la IP encontrada y conectamos el duckiebot con el computador a travéz de `export ROS_IP=...` y pegamos justo a continuación la IP copiada anteriormente. 
Se ejecuta `cd duckiebot_pc` y luego `python3 voice2text.py` para ejecutar el programa desde el PC.

Para que el duckiebot comienze a recibir instrucciones por voz se presiona el botón A en el Joystic. Cuando responde quack quack se le entrega la instrucción y se verá en la consola la instrucción recibida.

### 📄 Documentación adicional:
Por un lado, en el archivo duckiebot.py encontramos la librería `pyttsx3` hace hablar al programa por los altavoces, es decir Text to Speech. Luego se establecen las propiedades de esta voz como el idioma y el volumen al cual sale por el parlante.

Luego se define la función tiempo la cual al ingresar un ángulo, calcula el tiempo que le toma al duckiebot girar en ese ángulo. Para esto calculamos con cronómetro que el duckiebot se demora 1.5 segundos.

Luego definimos las propiedades del inicializador en el cual están los Subscribers los cuales reciben información de el Joystick, del tiempo en que queremos que gire, de la voz y de wikipedia.

