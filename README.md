# 🦆 Duckiebot interactivo
Este proyecto se está realizando para el curso **CD2201 Módulo Interdisciplinario** (Duckietown), desarrollado por [Nacho Dagach](https://github.com/ignaciodagachabugattas), [Pichi](https://github.com/pichiuwu) y [Max](https://github.com/maxfloresv).

Se trata de un robot que escucha instrucciones y responde mediante voz. Algunas de las interacciones adicionales que se buscan implementar son:

* Rastrear instrucciones para volver a un cierto punto en el pasado.
* Hacer que el Duckiebot pueda decir datos curiosos/chistes.
* Hacer que el Duckiebot sea capaz de bailar con una instrucción.
* Agregar palabras clave ocultas de interacción para realizar acciones secretas (*Easter Eggs*).

La información sobre los avances del proyecto puede ser encontrada en la [wiki](https://github.com/maxfloresv/robot_interactivo/wiki).

### 📄 Documentación adicional:
Para el desarrollo del proyecto, se ocupó un control de Xbox 360, y se usaron los siguientes botones:
* Palanca izq. para el giro (3 en la imagen).
* LT para retroceder (6 en la imagen). 
* RT para acelerar (9 en la imagen).

La referencia al control está en la función `callback_control` del [archivo principal](https://github.com/maxfloresv/robot_interactivo/blob/main/robot_interactivo.py#L81).

![d10437ca-81da-48db-a7a0-4c1173ad42e1](https://user-images.githubusercontent.com/45862114/199054372-978e232f-ea52-479f-8155-ffc665857241.png)


### 💻 Comandos útiles:
* Conexión remota del Duckiebot (para *testing* remoto): `cd duckietown` > `source vehicle_name.sh` > `./hotspot.sh on` > Conectarse a la red, cerrar y abrir otra terminal.
* `roslaunch ros_cap duckie_core.launch` para iniciar el core de ROS.
* `rostopic echo <nombre del tópico>` para ver información sobre un tópico de ROS (p. ej. la estructura de un mensaje).
* `rosrun <directorio> <archivo.py>` o `python <archivo.py>` para ejecutar un archivo (p. ej. `rosrun desafios_2022 Robot_interactivo.py`).
