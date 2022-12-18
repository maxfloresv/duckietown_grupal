# 🦆 Duckiebot interactivo
Este proyecto, desarrollado por [Nacho Dagach](https://github.com/ignaciodagachabugattas), [Pichi](https://github.com/pichiuwu) y [Max](https://github.com/maxfloresv), se está realizando para el curso **CD2201 Módulo Interdisciplinario** (Duckietown).

Recomendamos leas esta guia tras haber visto el video de demostración del proyecto en el siguiente [link](https://www.youtube.com/watch?v=ZXwcHaN6O-M&ab_channel=ProfeNacho), para que lo conozcas de primera mano, pata o ala, según corresponda.

Se trata de un robot que escucha instrucciones y responde mediante voz. Algunas de las interacciones adicionales que se buscan implementar son:

* Gama completa de movimientos comandados por voz.
* Hacer que el Duckiebot pueda decir datos curiosos/chistes.
* Hacer que el Duckiebot sea capaz de cantar y bailar con una instrucción.
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
**Necesitarás conexión estable a internet!!**

Para comenzar a utilizar el Duckiebot, se debe seguir lo siguiente. 

Primero se abre la consola en la cual se debe ingresar al Duckiebot utilizando `ssh -X duckiebot@duckiebot.local` y se ingresa la contraseña. Luego se abren las carpetas ingresando `cd \duckiebot\catkin_ws\src\desafios_2022\src`. Se ejecuta `roslaunch ros_cap duckie_core.local` y se ejecuta el programa en una nueva terminal desde el duckiebot con `rosrun desafios_2022 duckiebot.py`.

Para esto se abre una nueva terminal y se utiliza `ifconfig` la cual se usa para configurar y ver el estado de las interfaces de red en los sistemas operativos Linux. 

Luego copiamos la IP encontrada y conectamos el duckiebot con el computador a través de `export ROS_IP=...` y pegamos justo a continuación la IP copiada anteriormente. 
Se ejecuta `cd duckiebot_pc` y luego `python3 voice2text.py` para ejecutar el programa desde el PC.

Para que el Duckiebot comience a recibir instrucciones por voz se presiona el botón A en el Joystick. Cuando responde "quack quack" se le entrega la instrucción y se verá en la consola la instrucción recibida.

### 📄 Documentación adicional:
Por un lado, en el archivo **duckiebot.py**:

+ Encontramos la librería `pyttsx3` hace hablar al programa por los altavoces, es decir Text-to-Speech. Luego se establecen las propiedades de esta voz como el idioma y el volumen al cual sale por el parlante.

+ Luego, se define la función tiempo la cual al ingresar un ángulo, calcula el tiempo que le toma al Duckiebot girar en ese ángulo. Para esto calculamos con cronómetro que el Duckiebot se demora 1.5 segundos.

  Luego, definimos las propiedades del inicializador en el cual están los Subscribers los cuales reciben información del Joystick, del tiempo en que queremos que gire, de la voz y de wikipedia.
Además definimos los Publishers los cuales publican la información recibida a las ruedas y al parlante. Las siguientes son propiedades del estado inicial del Duckiebot y propiedades que se le asignan a cada instrucción para el momento de ejecutarlas.
Estas últimas están dadas por \[rapidez lineal, rapidez angular, tiempo de ejecución].

+ Se define la función `callback_control(self, msg)` la cual le asigna a cada botón de joystick y movimiento de las ruedas cambiando su velocidad lineal o angular. También se define el botón B como freno de emergencia y el freno si se presiona acelerar y retroceder al mismo tiempo.

+ Se utiliza la **distancia de Levenshtein** la cual nos entrega el número mínimo de operaciones que se deben hacer para transformar un string en otro. Éstas operaciones son agregar, eliminar o sustituir un elemento de este string. Definimos esta función para que si decimos \"avanzar" y el duckiebot detecta \"avanza", realice igual la acción. 

+ Se define `ejecutar_instrucción` la cual ejecuta una instrucción que se haya pedido, devolviendo False si no coincide con las funciones que se agregan y True si es la indicada y se ejecutó la acción. Ésto se realiza sólo si la palabra que se obtuvo tiene menos de 2 operaciones de diferencia con las indicadas (distancia de Levenshtein < 2) y para las instrucciones avanzar y retroceder pregunta de vuelta el tiempo en que se quiere que se realice la acción y se publica el tiempo ingresado. `sleep` es una función que permite detener un programa durante un tiempo y luego permitir que continúe sin alteraciones, en nuestro caso durante 5 segundos (para esperar el *input* por voz).

+ En la función `callback_voz` se extraen las propiedades de las acciones recibidas antteriormente y si se ejecuta la acción se detiene el Duckiebot. Además, se definen las propiedades de la función bailar, las cuales son tocar una canción random entre las definidas, avanzar y retroceder (instrucción de la 1 a la 5) y girar a la izquierda y a la derecha (instrucción de la 6 a la 9). Luego se publica este mensaje en las ruedas para ejecutar la acción. Luego de pasar el tiempo, se detiene el Duckiebot. 
  
+ La función `callback_tiempo` transforma a entero el tiempo ingresado y cambia a True la variable `valid` si se ingresa un numero entero positivo o cero.
+ La función `callback_wiki` recibe los datos obtenidos de Wikipedia y los imprime en la consola.

Por otro lado, en el archivo **voice2text**:
+ Luego de importar las librerías se establece el lenguaje español de la Wikipedia. Y se define `run = True` el cual define si corre o no el programa.

+ Se define el objeto de la clase en el cual se definen sus propiedades como los Publishers de voz a texto, de tiempo y de la búsqueda en Wikipedia, además de los Subscribers del Joystick y de la publicación de mensaje de Wikipedia. 
  El programa de voz está dado por `sr.Recognizer()` y se define las propiedades A y X como 0.
  
+ Se define el `callback` en la cual se encuentran las valiables globales run y active. Se utiliza el botón X como un término de emergencia del programa ya que deja de correr este último. Luego, se utiliza el botón A para ejecutar el micrófono. En el caso en que sea una llamada forzada de la función, la variable `auto = True` por lo que, por ejemplo si se recibe la instrucción avanzar, devolverá "Quack avanzar".

+ Dentro del bloque `try`, se intenta capturar las instrucciones de voz. Si ocurre algún error, pasa al bloque `except` donde el error en cuestión es imprimido en la consola (para *debuggear*).

+ Luego, `callback_control` a partir de la lista que recupera la posición de cada botón del Joystick, se le asigna la variable A al botón "A" del joystick y lo mismo con el botón "X".

+ La función `callback_req` recibe una instrucción del Duckiebot ("tiempo" ó "buscar"), lo que hace que se llame a la función que pide *inputs* de voz. Esta fue una manera de solucionar la comunicación entre el robot y el PC cuando se necesitaba algún dato (p. ej., el tiempo que el Duckiebot debía avanzar).

+ Por último, dentro de `main`, se define un Loop el cual ejecuta ROS salvo que se presione el botón X ya que éste transforma `run = False` y por lo tanto deja de correr el programa.

Adjuntamos además en este [link](https://drive.google.com/drive/u/1/folders/1Bi4bI9MDvnyQnD3DUqhRBa8QVUB1PoVH) el Google Drive utilizado por el equipo, donde podrás encontar material adicional, y el informe técnico y final del proyecto.


