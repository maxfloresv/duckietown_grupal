# 🦆 Duckiebot interactivo
Este proyecto se está realizando para el curso **CD2201 Módulo Interdisciplinario** (Duckietown).

Se trata de un robot que escucha instrucciones y responde mediante voz. Algunas de las interacciones adicionales que se buscan implementar son:

* Rastreo de instrucciones para volver a un cierto punto.
* Datos curiosos/chistes.
* Baila Duckiebot, baila.
* Palabras clave ocultas de interacción para realizar acciones secretas (*Easter Eggs*).


### 📄 Documentación adicional:
Para el desarrollo del proyecto, se ocupó un control de Xbox 360, con la siguiente disposición de botones:
* `axes[0]`: Palanca izq. para el giro (3 en la imagen).
* `axes[2]`: LT para retroceder (6 en la imagen). 
* `axes[5]`: RT para acelerar (9 en la imagen).

cuya referencia apunta a la función `callback_control` del [archivo principal](https://github.com/maxfloresv/robot_interactivo/blob/main/robot_interactivo.py#L81).

![d10437ca-81da-48db-a7a0-4c1173ad42e1](https://user-images.githubusercontent.com/45862114/199054372-978e232f-ea52-479f-8155-ffc665857241.png)


### 💻 Comandos útiles:
* `rosrun desafios_2022 Robot_interactivo.py`: Ejecuta el programa principal `robot_interactivo.py`.
> ⚠️ Este comando se debe ejecutar dentro del Duckiebot.
* `python <archivo>`: Ejecuta algún archivo dentro del PC, como `voice2text.py` o `voice2voice.py`.
