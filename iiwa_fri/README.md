# 1. Requirements
## Ubuntu 16.04 WSL
Installation:
- Download and follow the instructions inside the Description section.
```
https://www.microsoft.com/en-us/p/ubuntu-1604-lts/9pjn388hp8c9#activetab=pivot:overviewtab
```
- Install XLaunch 
```
https://sourceforge.net/projects/xming/
```
Alternatives: 
- Native Ubuntu 16.04
```
https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop-1604#0
```
Recommendations:
- Change blue color: Right click > Properties > Colors > Screen text > Select blue to change > Change values to [59, 120, 255]
- Install terminator

## ROS Kinetic
- Install ros-kinetic-desktop-full

# 2. Installation

```
roscd
catkin_make
```
# 3. FRI Configuration

## Problema de incompatibilidad con iiwa_stack 1.2.5
La versión 1.2.5 de iiwa_stack utiliza el puerto KONI. Para poder utilizarlo y comunicarse con Java, la guía de Salvo Virga para el iiwa_stack 1.2.5 explicaba cómo permitirle a Windows utilizar el puerto KONI. Esto hace que el puerto KONI no pueda ser utilizado por RTOS, y por lo tanto no funcione FRI. Estas instrucciones ya no están en el github de Salvo Virga. 
En ambos casos, es necesario conectar el cabinet a una pantalla, teclado y ratón para utilizar el Windows sobre el que se está ejecutando el programa SmartHMI, que es la parte gráfica de KRC (KUKA Robot Controller). 
### Configurar puerto KONI para utilizar el iiwa_stack v1.2.5 
  - Enchufar pantalla, ratón y teclado a la controladora e iniciar sesión. 
  - Apagar KRC: Click derecho en el icono verde abajo a la derecha al lado del reloj, “Stop KRC”. 
  - Abrir cmd.exe y escribir: ```C:\KUKA\Hardware\Manager\KUKAHardwareManager.exe -assign OptionNIC - os WIN```. Esto asignará el puerto KONI a Windows, y dejará de estar disponible para FRI. 
  - Reiniciar el ordenador, abrir el panel de conexiones, elegir la red nueva (no la red que hay con el IIWA) y abrir las propiedades de Internet Protocol Version 4 (TCP/IPv4). Para diferenciar ambas redes, la del IIWA debería tener fijada la IP a 192.168.0.1 (no tocar esta IP), y la otra debería aparecer automáticamente en la opción “Obtain an IP address automatically” cuando se acaba de asignar el puerto KONI a Windows. 
  - En la ventana de propiedades de la red nueva, establecer la IP 160.69.69.69, máscara de subred 255.255.255.0 y puerta de enlace predeterminada 160.69.69.1. 
   - Conectar el ordenador con Ubuntu/ROS al puerto KONI. Establecer en este ordenador la IP fija 160.69.69.100, que actuará como ROS Master. 
### Configurar puerto KONI para utilizar FRI 
   - Abrir cmd.exe como administrador 
   - Ejecutar “C:\KUKA\Hardware\Manager\KUKAHardwareManager.exe -assign OptionNIC -os RTOS” 
   - Reiniciar Windows 
## Configuración de Sunrise Workbench 
Se puede hacer sobre un proyecto nuevo de Sunrise o sobre uno existente 
1. Asegurarse de que está instalada la opción FRI para Sunrise Workbench. 
   a. En Sunrise Workbench > Ayuda > Install New Software > Añadir > Archive b. Añadir la carpeta con los .zips de instalación para el Sunrise Workbench 1.16, que se encuentra en ROBOESPAS_COLABORATIVE > Recursos > Configuracion > Sunrise Cabinet > Sunrise.OS.1.16.0_B7_C480415 > Sunrise.OS.1.16.0_B7_C480415 > options c. Seleccionar KUKA Connectivity, Siguiente d. Si ya está instalado dará un error y dice que la instalación será ignorada porque ya está instalado, simplemente hacer clic en Cancelar. 
2. Asegurarse de que las 3 primeras líneas del archivo SafetuConfiguration.sconf están desmarcadas 
3. En StationSetup.cat, asegurarse de que las dos opciones de Fast Robot Interface Extension y Fast 
Robot Interface Extension Example Application están marcadas. 
4. Al marcar estas opciones, aparecerán dos cosas nuevas en nuestro proyecto: 
   a. Las aplicaciones de ejemplo que habrá que ejecutar en el SmartPad para probar los ejemplos 
y en las que basaremos posteriores aplicaciones utilizando FRI. Aparecerán en: ProyectFRI\examples\com\kuka\connectivity\fri\example 
   b. Las clases en C++ que proporciona KUKA para poder comunicarse con la controladora a través de FRI desde otro ordenador (Cliente/Remoto) en C++. Aparecerán en: ProyectFRI\FastRobotInterface_Client_Source\FRI-Client-SDK_Cpp.zip. Habrá que copiarlas y pegarlas en el ordenador que vayamos a utilizar como Cliente/Remoto, y se pueden borrar del proyecto de Sunrise posteriormente. Si se necesita de nuevo el .zip, se puede desmarcar las casillas de StationSetup.cat/Software y volver a marcarlas, y volverá a aparecer. 
* Además, aparecen unos archivos de Java en ProyectFRI\FastRobotInterface_Client_Source\ FRI-Client-SDK_Java, que no utilizaremos porque sólo sirven para comunicarse con la controladora a través de FRI desde otro ordenador (Cliente/Remoto) en Java. Y en nuestro caso, lo haremos a través de C++. No confundir, estos archivos NO son aplicaciones que se puedan ejecutar en el SmartPad. 
5. Configurar IPs en Sunrise Workbench 
   a. En StationSetup.cat > Configuración > KUKA Option Network Interface > IP, fijar 
192.170.10.2 si no está ya fijada b. En todas las aplicaciones de ejemplo que han aparecido tras activar la opción de FRI para 
este proyecto, modificar la IP a la IP que estableceremos posteriormente en el Remoto/Cliente (el Ubuntu que ejecutará los programas de C++). Modificar _clientName y establecer “192.170.10.200” y salvar en: 
      - FRIIOApp.java 
      - LBRJointSineOverlay.java 
      - LBRTorqueSineOverlay.java 
      - LBRWrenchSineOverlay.java 
      - TransformationProvider.java 
6. Configurar puertos en Sunrise Workbench. Ambos puertos deben ser iguales para que funcione. Sino no se conectarán. (¿Por qué?) 
   a. En todas las aplicaciones de ejemplo, establecer los puertos utilizados tanto para el Client como para el Controller. Añadir en la función initialize 2 líneas nuevas al final: 
    ```
    _clientPort=30200;
    _controllerPort=30200;
    ```
   b. Añadir estas variables como variables de la clase privadas al principio de la clase private int _clientPort; private int _controllerPort; 
   c. Modificar friConfiguration para que utilice estos puertos. Añadir tras la línea en la que se crea friConfiguration lo siguiente: 
    ``` 
    friConfiguration.setPortOnController(_controllerPort); 
    friConfiguration.setPortOnRemote(_clientPort); 
    ```
   d. Modificar la información que sale por el SmartPad para mostrar estos puertos. 
    ```
    getLogger().info("Creating FRI connection from controller port " + friConfiguration.getPortOnController() + " to " + friConfiguration.getHostName() + ":" + friConfiguration.getPortOnRemote() ); 
    ```
   e. Aplicar estos cambios a todos los ejemplos y salvar. 
   f. Sincronizar con la controladora para aplicar los cambios. 
## Compilación de ejemplos en Ubuntu / C++ 
1. Configurar una red cableada nueva con IP fija 192.170.10.2, llamarla FRI por ejemplo. Conectarse a esa red y desconectar de la red WiFi por si acaso. 
2. Descomprimir la carpeta FRI-Client-SDK_Cpp.zip 
3. Hay un error en el Makefile, que habrá que arreglar. Abrir *FRI-Client-SDK_Cpp/build/GNUMake/Makefile* con un editor de texto. En la penúltima línea que sigue a ```EXAMPLE_DIRS =```, añadir una barra al final. La línea quedará así: 
 ```$(EXP_DIR)/SimulatedTransformationProvider \``` 
4. Compilar todo: cd FRI-Client-SDK_Cpp/build/GNUMake make 
5. Ejecutar cada ejemplo: **CUIDADO! El robot se moverá** 
Para ejecutar cada ejemplo podemos ir a la carpeta examples, y en cada carpeta interior habrá un ejecutable para cada ejemplo. Si ejecutamos ./NombreEjemplo help nos saldrá una ayuda, pero IMPORTANTE! La ayuda está MAL, pone que se le puede pasar opcionalmente la IP y puerto del Remote/Client (el propio ordenador desde el que se ejecuta ese ejecutable, pero en realidad hay que pasarle la IP y Puerto de la controladora (el cabinet). Por ejemplo, para el ejemplo IOAccess 
```
cd FRI-Client-SDK_Cpp/examples/IOAccess 
./IOAccess “192.170.10.2” 30200
```
Si funciona, en la terminal de Ubuntu aparecerá: 
```
IOAccessClient initialized: Enter IOAccess Client Application Exit IOAccess Client Application 
```
Y en el SmartPad: 
```
Creating FRI connection from controller port 30200 to 192.170.10.200:30200 SendPeriod: 5ms | ReceiveMultiplier: 1 Connection to Client established Enable clock Do something ... Disable clock Close connection to client 
```
6. Se pueden modificar los ejemplos y recompilar individualmente haciendo make clean y make de 
nuevo en la carpeta de cada ejemplo. 

### Resumen IPs y Puertos 
1. Remote/Client: 192.170.200:30200 
2. Controller: 192.170.10.2:30200 
