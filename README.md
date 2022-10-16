# Laboratorio 4 de Robótica
## Universiad Nacional de Colombia
## 2022-2
***
### Autores
- Andrés Holguín Restrepo 
- Julián Andrés Caipa Prieto
### Profesores encargados
- Ing. Ricardo Emiro Ramírez H.
- Ing. Jhoan Sebastian Rodriguez R.
***
### Introducción

### Cinemática directa


#### Mediciones del pincher
Inicialmente es pertinente caracterizar el Pincher para poder comparar los resultados reales con resultados teóricos previamente obtenidos. Para ello se hace uso de Matlab y el Toolbox de Peter Corke con cálculos de cinemática directa por parámetros de Denavit-Hartenberg. 

Se requiere obtener ciertas mediciones del Pincher, las cuales están asociadas a las distancias entre ejes articulares, ya que son necesarios para formular la tabla de parámetros que describen el Robot. La siguiente imagen muestra las medidas requeridas:

![robot medidas](https://github.com/aholguinr/Lab4_Robotica_Caipa_Holguin/blob/main/Imagenes/robot.png?raw=true)

Las cuales resultan como L1=40.6 mm, L2=L3=107 mm y L4=69.5 mm. Cabe destacar que desde el Pincher tiene el eje de la primera articulación a una cierta altura debido a la base que tiene, sin embargo, acá no se tiene en cuenta, por ello el robot no tiene en cuenta como medida el tamaño de la base. 

#### Tabla DHstd
Ahora bien, se requiere ubicar los marcos de referencia para extraer la tabla, siguiendo la convención DH estándar, obteniendo:

![robot marcos](https://github.com/aholguinr/Lab4_Robotica_Caipa_Holguin/blob/main/Imagenes/dhrobot.png?raw=true)

Con lo que se puede plantear la siguiente tabla de parámetros DHstd:

| Theta    | d | a | alpha | Offset |
| :----:   | :----:      | :----:     | :----:     | :----:     |
|Theta 1         |  L1        |    0     |   90     |    0°     | 
|Theta 2         |    0      |     L2    |    0°    |    90°     |   
|Theta 3         |    0      |     L3   |  0°     |    0°     | 
|Theta 4         |      0    |     0    | 90°     |     90°    |   
|Theta 5         |      L4    |     0    | 0°        |     0°    |  

#### ToolBox

Es requerido graficar el robot en las posiciones que presenta la siguiente tabla:

| Pose     | Articulación 1 |Articulación 2 |Articulación 3 |Articulación 4 |Articulación 5 |
| :----:   | :----:      | :----:     | :----:     | :----:     | :----:     |
|Home      | 0°         |  0°       |   0°     |    0°     |      0°   |
|1         |  -20°        |    20°     |   -20°     |    20°     |   0°      |
|2         |    30°      |     -30°    |    30°    |    -30°     |     0°    |
|3         |    -90°      |      15°   |  -55°      |    17°     |    0°     |
|4         |      -90°    |     45°    |-55°        |     45°    |     10°    |

Para lo cual, contando con los parámetros DHstd, se hace uso del comando SerialLink del toolbox de Peter Corke. Al graficar el robot en la primera posición, la cual es Home, se obtiene lo siguiente:

![robot home](https://github.com/aholguinr/Lab4_Robotica_Caipa_Holguin/blob/main/Imagenes/Home.png?raw=true)

De dónde se pueden destacar varias cosas, primero, al utilizar los comandos del toolbox de Peter Corke resulta un robot con la misma orientación de Home como la planteada inicialmente, por lo cual se verifica además la correcta definición de los parámetros DHstd. Otra cosa es que el último marco de referencia tiene la orientación de tipo _noa_, ya que el eje z cumple la función de eje de aproximación, mientras que el eje y cumple las veces del de apertura y el eje x el normal. Si esto no hubiera sido así, se hubiera podido corregir haciendo uso del comando Tool, el cual toma el robot y cambia la orientación o posición (o ambas) del TCP respecto al último marco de referencia definido con el comando Link, de la siguiente forma:

```
Robot.tool=MTH_TCP->o_6
```
Otra cosa es que cada una de las articulaciones tiene el marco de referencia graficado, esto se hace con fin de verificar los parámetros y la correcta orientación de los marcos respecto a los movimientos que se ejercen en la vida real, para esto se hace uso del siguiente código, en donde la primera línea después del _Hold on_ permite graficar el último marco de referencia, el cual estará al graficar el robot en todas las posiciones para hacerse una idea de cómo queda la herramienta al mover las articulaciones, mientras que las demás sirven para graficar los otros marcos haciendo uso de las matrices intermedias de la cinemática directa, que en el toolbox se definen como matrices A y que se evaluan en un vector _q_ dado que tiene los valores solicitados de los ángulos de las articulaciones.

```
Robot_q_1.plot(q_1,'view',[-30 30], 'jointdiam',2);
hold on 
trplot(eye(4),'rgb','arrow','length',15,'frame','0')
axis([repmat([-40 40],1,2) 0 40])
Robot_q_1.teach()
M= eye(4);
for i=1:Robot_q_1.n-1
    M = M * Li(i).A(q_1(i));
    trplot(M,'rgb','arrow','frame',num2str(i),'length',15)
end
hold off
```

Por último, se puede obtener la matriz de transformación del sistema de muchas formas, obteniendo las matrices A con el toolbox y multiplicandolas todas entre ellas, haciendo uso del comando A modificado, en la cual se le ingresa un arreglo con los números de las articulaciones entre las cuales se quiere calcular la matriz y otro arreglo con los valores _q_ para dichas articulaciones, algo como:

```
Robot.A([1,2,3,4,5,6],[theta_1 theta_2 theta_3 theta_4 theta_5 theta_6])
``` 

O el método utilizado en este caso, que es, contando con la tabla de parámetros DHstd, simplemente se hace cada matriz A, que se hace multiplicando cuatro matrices, dos de rotación y dos de traslación, en el siguiente orden:

```
A_i=trotz(theta_i+offset)*transl(0,0,d_i)*transl(a_i,0,0)*trotx(alpha_i)
```
Y finalmente multiplicarlas todas entre ellas. De forma simbólica se tiene la siguiente matriz:
```math
a^2+b^2=c^2
```


### Desarrollo código python

A continuación se muestra todo el procedimiento realizado para el desarrollo del .py que efectua los movimientos del pincher.

#### Datos iniciales

Ahora bien, lo primero que se debe establecer es que hay ciertas limitantes que se dieron. Debido a que en código uno lo que puede realizar es cambiar los valores de registro del pincher, en este caso de 0 a 1023 para los registros de posición que van de un rango de 0° a 300°, es necesario conocer los valores puntuales de los registros para obtener estas posiciones. Esto se logra mediante el uso del Dynamixel Wizard. AL llevar el Picher a la posición final que queremos, se pueden extraer los valores de los registros directamente y almacenarlos para su implementación futura. Dicho esto, a continuación se muestran los valores de los registros teniendo en cuenta las poses deseadas y el Home previamente establecido.


| Pose     | Registro M1 |Registro M2 |Registro M3 |Registro M4 |Registro M5 |
| :----:   | :----:      | :----:     | :----:     | :----:     | :----:     |
|home      |514          |510         |818         |512         |512         |
|1         |444          |575         |751         |580         |512         |
|2         |614          |410         |922         |409         |512         |
|3         |205          |560         |630         |570         |512         |
|4         |205          |660         |629         |666         |546         |

De este modo, es posible asignar los valores de los registros al valor que corresponda equivalente al ángulo esperado. Otra aclaración es que en estos pincher, se tiene un rango de los servos de 0° a 300°, donde su posición en home está en realidad en 150°, esto quiere decir que, al tomar como referente este home, tiene unos límites de giro de +150° y -150°. Además, como se mostró en secciones anteriores, el home de la tercera articulación tiene un offset de +90°, esto quiere decir que su home está en 240°, lo cual implica que tiene un rango de movimiento de +60° y -240°. Esto es una simple aclaración, porque para las poses deseadas, solo se aumenta un máximo de +30° en esta articulación, permitiendole al pincher alcanzar todas las poses.

Ya teniendo esto establecido, se puede empezar con el código realizado en python.


#### Python
Para el desarrollo de esta práctica se decidió trabajar con python en vez de Matlab ya que se considera de este modo más facil la configuración. Además, se diseñan rutinas donde se envía el pincher a home al inicio, y luego se dirije a la pose deseada. También se realiza la lectura de datos de los ángulos reales del pincher para confirmar que esté ubicado correctamente, junto con el cálculo del error RMS asociado a cada pose.

Cabe resaltar que se adjunta la carpeta de dynamixel_one_motor de donde se ejecutan los archivos


en archivo .py de este repositorio se encuentra con comentarios todo el código. A continuación solo se realizan unas explicaciones de las rutínas involucradas en dicho archivo.

##### Lectura de sensores

Para realizar la lectura de los sensores de posición, se debe crear un subscriber a los estados de articulación, el cual se realiza mediante una función llamada listener y de la función callback que se ejecuta cada vez que el subscriber recibe mensajes:
```
def callback(data):
    global PosActual
    PosActual=np.multiply(data.position,180/pi)
    PosActual[2]=PosActual[2]-90


def listener():
    rospy.init_node('joint_listener', anonymous=True)
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState,callback)
```

Como se puede ver, se inicia el nodo con el valor anonymous=True en caso de que se requiera agregar más nodos. Y, según la función callback, se entiende que la variable global PosActual es una lista de 5 valores, cada uno con la lectura de los ángulos en radianes vistos desde el home preestablecido del pincher. Debido a que nuestra articulación 3 está desfasada +90° por el offset de home establecido, estos 90° se deben restar de este valor. Dicho esto, se tiene una variable global que siempre se va a actualizar cuando el SUbscriber recibe nuevos mensajes de posición angular de las articulaciones.


#####  Movimiento de articulaciones


Para el movimiento de articulaciones se tiene generada la función jointCommand, definida a continaución:
```
def jointCommand(command, id_num, addr_name, value, time):
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))
```
Como se puede ver, es general para cualquier comando de dynamixel, no solo para movimientos. COn tal de tener el nombre adecuado del comando, este se transmite y se ejecuta. En este caso, para generar movimientos, es con el comando 'Goal_Position', el cual lo que hace es cambiar el registro con este mismo nombre, que acepta valores enteros de 0 a 1023, motivo por el cual anteriormente se determinaron estos valores. Una forma de aproximarse y facilitar la entrada de datos es linealizar este rango de 0 a 1023 al rango de movimiento -150° a 150°. Sin embargo, al hacer las pruebas por este método, no siempre se obtenian los resultados deseados, por lo que la exactitud disminuía.

Con base a esto, lo que se realiza para actualizar las posiciones de las articulaciones, empezando por la base hasta el extremo, es realizar el siguiente ciclo:

```
def movPartido(j,Goal,Actual):
    N=5
    delta=((Goal-Actual)/N)
    for i in range(N):
        jointCommand('', (j+1), 'Goal_Position', int(Actual+delta*(i+1)), 0.5)
        time.sleep(0.1)


print('Empezar rutinas\n')
for i in range(5):
    print('Moviento eslabon: '+str(i+1))
    movPartido(i,CasoA[i],PosHome[i])
print('Finalizada la rutina.')
printL(PosActual,CasoDeg)
```

Dicho esto, se realizan los movimientos de de manera individual y con movimientos intermedios para no generar movimientos bruscos en el Pincher, esto con la función movPartido, que uno puede indicar cuantos subrecorridos se van a realizar, en este caso 5, incluyendo hasta llegar a la posición final.

De este modo, se pueden realizar todas las rutinas deseadas, desde ir a home, hasta a una posición arbitraria asociada a una lista de 5 valores enteros de 0 a 1023.



##### Limitación de torques

POr último, se realizan limitaciones de torque para no generar movimientos tan bruscos en el Pincher, sin embargo, no puede ser un torque tan bajo debido a que pueden haber casos donde el motor entraría en overload ya que se requiere un torque mayor al que se le está permitiendo. Esto se realiza mediante la el nombre de función 'Torque_Limit' con la misma metodología del cambio del Goal_Position, donde se asignan valores de torque máximo de 0 hasta 1023 de la siguiente forma:
 
```
for i in range(5):    
            jointCommand('', (i+1), 'Torque_Limit', Torques[i], 0)
```

Torques es un arreglo con los valores asignados.


#### Ejecución 

Con todo y lo anterior, se debe ejecutar el archivo lab4.py. Luego, al usuario le aparece en pantalla las 4 opciones a ejecutar y debe seleccionar alguna pulsando un número de 1 a 4 y darle enter. Luego empieza la rutina de ir al home del robot sin intermedios y luego a la rutina establecida con 4 puntos intermedios para cada articulación. En consola aparece el punto de ejecución en el que está, y el error de posición angular del home y de la posición final. A continuación se muestra el ejemplo de esta impresión inicial en consola:
 
 
 
![Consola inicial](https://github.com/aholguinr/Lab4_Robotica_Caipa_Holguin/blob/main/Imagenes/inicio.png?raw=true)

### Resultados finales

#### Video

Click en la siguiente página para dirigirse al video:

[![video](https://i9.ytimg.com/vi_webp/i1MiGquxebQ/mqdefault.webp?sqp=COCCqJoG&rs=AOn4CLCMm6SIbKCXUOZCQjj9puunpE8E6g)](https://www.youtube.com/watch?v=i1MiGquxebQ)

#### Caso 1

![Resultados consola caso 1](https://github.com/aholguinr/Lab4_Robotica_Caipa_Holguin/blob/main/Imagenes/SS1.png?raw=true)

![Imag 1](https://github.com/aholguinr/Lab4_Robotica_Caipa_Holguin/blob/main/Imagenes/VP1.png?raw=true)

Comparándo este resultado con el que se generó en el Toolbox se puede identificar que son equivalentes.

#### Caso 2

![Resultados consola caso 1](https://github.com/aholguinr/Lab4_Robotica_Caipa_Holguin/blob/main/Imagenes/SS2.png?raw=true)

![Imag 2](https://github.com/aholguinr/Lab4_Robotica_Caipa_Holguin/blob/main/Imagenes/VP2.png?raw=true)

Comparándo este resultado con el que se generó en el Toolbox se puede identificar que son equivalentes.

#### Caso 3

![Resultados consola caso 1](https://github.com/aholguinr/Lab4_Robotica_Caipa_Holguin/blob/main/Imagenes/SS3.png?raw=true)

![Imag 3](https://github.com/aholguinr/Lab4_Robotica_Caipa_Holguin/blob/main/Imagenes/VP3.png?raw=true)

Comparándo este resultado con el que se generó en el Toolbox se puede identificar que son equivalentes.

#### Caso 4

![Resultados consola caso 1](https://github.com/aholguinr/Lab4_Robotica_Caipa_Holguin/blob/main/Imagenes/SS4.png?raw=true)

![Imag 4](https://github.com/aholguinr/Lab4_Robotica_Caipa_Holguin/blob/main/Imagenes/VP4.png?raw=true)

Comparándo este resultado con el que se generó en el Toolbox se puede identificar que son equivalentes.

 
 
 
