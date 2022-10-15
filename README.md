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

#### Tabla DHstd

#### ToolBox


| Pose     | Articulación 1 |Articulación 2 |Articulación 3 |Articulación 4 |Articulación 5 |
| :----:   | :----:      | :----:     | :----:     | :----:     | :----:     |
|home      | 0°         |  0°       |   0°     |    0°     |      0°   |
|1         |  -20°        |    20°     |   -20°     |    20°     |   0°      |
|2         |    30°      |     -30°    |    30°    |    -30°     |     0°    |
|3         |    -90°      |      15°   |  -55°      |    17°     |    0°     |
|4         |      -90°    |     45°    |-55°        |     45°    |     10°    |



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

### Resultados

#### Video

[![Alt text](https://i9.ytimg.com/vi_webp/i1MiGquxebQ/mqdefault.webp?sqp=COCCqJoG&rs=AOn4CLCMm6SIbKCXUOZCQjj9puunpE8E6g)](https://www.youtube.com/watch?v=i1MiGquxebQ)




#### Caso 1

Imágen caso 1:



#### Caso 2

Imágen caso 2:


#### Caso 3

Imágen caso 3:


#### Caso 4

Imágen caso 4:


 
 
 
