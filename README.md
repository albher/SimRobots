# SimRobots
Colaboración entre "SimScape Multibody" y "Robotic Systems Toolbox"

## Descripción
* Los robots de la estación se modelan con "SimScape Multibody"
* Las trayectorias se obtienen con "Robotic Systems Toolbox"
* Se ha usado la versión de Matlab 2023a, importante para los ficheros de Simulink

## Instrucciones para descargar ficheros y usar el programa:
* Bajarse el fichero __*.zip__ del proyecto
* Descomprimir el fichero en un directorio
* Abrir Matlab e ir al directorio del proyecto
* Crear un proyecto que englobe las carpetas que se desean usar

## Instrucciones para crear un proyecto local:
* Ir al directorio del proyecto
* Abrir un proyecto de Matlab: **HOME>New>Project>From Folder**
* Añadir las carpetas que se deseen usar:
  
  01_Teoría <br> 
  05_Cinemática <br>
  10_Dinámica <br>
  15_Médica <br>
  20_Comunicaciones <br>
  25_Móvil <br>
  30_Visión <br>
  35_Humanoides <br>
  50_Clase_Otros <br>
  APP <br>
  Biblioteca <br>
  Mfiles <br>
  RobMat <br>
  Robots (carpeta y subcarpetas) <br>
  RobSubSys <br>

* Ya tenemos el path de todas las carpetas y podemos usarlas sin problema. 
* La siguiente vez, el proyecto ya está definido, y solo hay que ejecutar el mismo picando sobre él.

## Instruciones para crear un proyecto directamente desde **git**:
* Abir proyecto **HOME>New>Project>From Git**
* Introducir la dirección del proyecto **https://github.com/albher/SimRobots.git**
* El proyecto se descarga en la carpeta deseada, pero los ficheros solo son de lectura.

## Importar nuevos robots en formato URDF y DH:
* "Robotic Systems Toolbox" dispone de otros ficheros de robot URDF en https://github.com/mathworks/Industrial-Robots-Simscape/tree/master
* La librería "ARTE" dispone de otro muchos robot en formato DH en https://github.com/4rtur1t0/ARTE/tree/master
