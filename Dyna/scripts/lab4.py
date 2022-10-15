from cmath import pi
import numpy as np
import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import DynamixelCommand

__author__ = "Andres Holguin"
#Arreglo de torques
Torques=[500,400,400,400,400]


#Ángulos deseados, el home y los 4 casos a evaluar
Deghome=[0,0,0,0,0]
Deg1=[-20, 20, -20, 20, 0]
Deg2=[30,-30, 30, -30, 0]
Deg3=[-90, 15, -55, 17, 0]
Deg4=[ -90, 45, -55, 45, 10]


#FUnción print para mostrar en consola los ángulos de los casos.
def printLB(pos):
    print('Posicion eslabones:\n')
    for i in range (len(pos)):
        print(str(i+1)+': '+"%.2f" % pos[i]+'°\t', end = ' ')
    print('\n')


#Imprimir los 4 casos posibles.
print('Ingrese caso a realizar:\n\n')
print('Caso 1. ',end = ' ')
printLB(Deg1)
print('Caso 2. ',end = ' ')
printLB(Deg2)
print('Caso 3. ',end = ' ')
printLB(Deg3)
print('Caso 4. ',end = ' ')
printLB(Deg4)

#Obtener del usuario un valor 
caso=int(input())

#Valores análogos de la posición home y los casos. Se obtuvieron desde dynamixel_wizard
PosHome=[514,510,818,512,512]
pos1An=[444,575,751,580,512]
pos2An=[614,410,922,409,512]
pos3An=[205,560,630,570,512]
pos4An=[205,660,629,666,546]

#Arreglo de los casos 1 a 4
posicionesAn=[pos1An,pos2An,pos3An,pos4An]
posicionesDeg=[Deg1,Deg2,Deg3,Deg4]

#Asigna qué caso se va a ejecutar.
CasoA=posicionesAn[caso-1]
CasoDeg=posicionesDeg[caso-1]
#Un arreglo en ceros donde se almacenarán las posiciones de los ángulos reales.
PosActual=[0,0,0,0,0]


#Función que cambia valores de registros de los motores del Pincher.
def jointCommand(command, id_num, addr_name, value, time):
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

#Función callback que se llama en el listener. Cambia la variable global de la posición de los ángulos de los motores.
#Se realiza el ajuste a grados y a la posición home que se estableció
def callback(data):
    global PosActual
    PosActual=np.multiply(data.position,180/pi)
    PosActual[2]=PosActual[2]-90



#Imprime la posición real de los ángulos de los motores.
def printL(real,teorico):
    print('\nÁngulos motores:\n')
    for i in range (len(real)):
        print(str(i+1)+': '+"%.2f" % real[i]+'°\t', end = ' ')
    Verror=np.sqrt(np.mean(np.subtract(teorico,real)**2))
    print('\n\n'+'Error RMS: '+"%.2f" % Verror+'°\n')
    
#Función que genera el subscriber para obtener los estados de las articulaciones
def listener():
    rospy.init_node('joint_listener', anonymous=True)
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState,callback)

#Rutina de movimiento con puntos intermedios. Uno define el número de movimientos N para llegar a un punto.
#Se ejecuta en un ciclo for hasta llegar al punto final.
def movPartido(j,Goal,Actual):
    N=5
    delta=((Goal-Actual)/N)
    for i in range(N):
        jointCommand('', (j+1), 'Goal_Position', int(Actual+delta*(i+1)), 0.5)
        time.sleep(0.1)



#Main
if __name__ == '__main__':
    try:
        #Activar el subscriber.
        listener()

        #Definir los límites de torque de los motores.
        for i in range(5):    
            jointCommand('', (i+1), 'Torque_Limit', Torques[i], 0)
            
        #Rutina para ir al home.
        print('Ir a home\n')
        for i in range(5):
            jointCommand('', (i+1), 'Goal_Position', PosHome[i], 1)
            print('Moviento eslabon: '+str(i+1)+'\n')
            time.sleep(0.5)
        print('En home\n')

        #Imprimir la posición real respecto al home.
        printL(PosActual,Deghome)
        
        #Realizar la rutina de movimiento.
        print('Empezar rutinas\n')
        for i in range(5):
            print('Moviento eslabon: '+str(i+1))
            movPartido(i,CasoA[i],PosHome[i])
        print('Finalizada la rutina.')

        #Imprimir la posición deseada respecto a la teórica.
        printL(PosActual,CasoDeg)
    except rospy.ROSInterruptException:
        pass



