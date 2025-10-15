#!/usr/bin/env python
import rospy
import smach
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

#Estos dos son los callbacks de los topics necesarios para el programa

# Callback del laser
def scan_callback(msg, userdata):   
    userdata.laser = msg
    userdata.laser_available = True

# Callback de la odometria (para saber posicion del robot
def odom_callback(msg, userdata):
    userdata.odom = msg
    userdata.odom_available = True


#A partir de aqui definimos los estados y el comportamiento de cada uno de ellos

class BuscarOponente(smach.State):  #Primer estado, buscamos a nuestro oponente 
    def __init__(self):
        smach.State.__init__(self,outcomes=['oponente_detectado', 'continuar_busqueda', 'borde_detectado'],input_keys=['laser', 'laser_available', 'odom', 'odom_available'])

    def execute(self, userdata):
        global pub
        rospy.loginfo("Estado: BUSCAR_OPONENTE")

        move = Twist()  #Defimios el movimiento como twist
        rate = rospy.Rate(4)  #Tenemos que pasar una frecuencia para cuando publiquemos en los topics de movimiento

        # Giro lento para buscar
        move.linear.x = 0.0  #No se mueve lineal
        move.angular.z = 0.6  #Se mueve en angular para poder buscar 
        start_time = rospy.Time.now()  #Definimos el tiempo cuando empieza el combate como tiempo inicial 

        while not rospy.is_shutdown():   #Publicador
            pub.publish(move)  #Publicamos el movimiento que acabanos de definir
            rate.sleep()  #Lo ponemos en sleep 

            # Si hay datos del laser disponibles
            if userdata.laser_available:
                ranges = np.array(userdata.laser.ranges)  #Definimos los rangos del laser
                n = len(ranges)

                # Dividir laser en 4 sectores
                p1 = np.nanmin(ranges[:n//4])
                p2 = np.nanmin(ranges[n//4:n//2])
                p3 = np.nanmin(ranges[n//2:3*n//4])
                p4 = np.nanmin(ranges[3*n//4:])

                d = min(p1, p2, p3, p4)  #Pasamos la distancia de delante como una tupla y cogemos el minimo, esa sera la distancia
                de = min(p1, p2)  #Hacemos lo mismo pero con la derecha y con los 2 primeros cuadrantes
                iz = min(p3, p4)  #hacemos lo mismo pero con la izquierda y con los 2 ultimos cuadrantes
		print("Distancia delante:", d, "Distania izquirda:", iz, "Distancia derecha:", de)  #De esta forma tenemos las distancias definidas, vamos a imprimirlas mientras buscamos al oponente
                # Si detecta algo cerca (enemigo)
                if d < 3.0:  #Si la distancia general es menor de 3, acabamos de encontrar a nuestro oponente, por lo tanto transicionamos de estado
                    print("Oponente detectado a:",d, "metros")  #Mostramos la distancia a la que encontramos nuestro oponente
                    move.angular.z = 0.0  #Dejamos de dar vueltas
                    pub.publish(move)  #Publicamos el cambio que acabamos de hacer
                    return 'oponente_detectado'

            # Si han pasado mas de 10 segundos girando, avanzar un poco
            if (rospy.Time.now() - start_time).to_sec() > 10.0:   #Si al tiempo de ahora le restamos el tiempo inicial y damas de 10 segundos, quiere decir que llevamos mas de 10 segundos dando vueltas
                rospy.loginfo("No se encontro oponente, avanzando hacia adelante")  #Por lo tanto nos movemos un poo hacia adelante para explorar zonas "desconocidas"
                move.angular.z = 0.0  #Dejamos de movernos en angular
                move.linear.x = 0.3 #Nos movemos un poco en lineal
                for _ in range(8):  # avanza unos segundos (le ponemos un for que itera 8 veces
                    pub.publish(move)  #Publicamos el movimiento que acabamos de hacer
                    rate.sleep()  #Lo dejamos en sleep
                move.linear.x = 0.0  #Cuando terina el for vuelve a pararse linealmente para que empiexe a buscar angularmente

            #Como acabamos de movernos con el robot vamos a comprobar que no nos caigamo del ring
            if userdata.odom_available: #Si la odometria esta dsponible
                x = userdata.odom.pose.pose.position.x  #Definimos la posicion x
                y = userdata.odom.pose.pose.position.y  #Definimos la posicion y
                r_ring = 2.0  #Damos el radio del ring 
                margen = 0.3 #Ponemos como margen 0,3m
                dist_centro = np.sqrt(x**2 + y**2)  #Por lo tanto podemos calcular la disranca al centro del ring
                if dist_centro > (r_ring - margen):  #Si la distancia al ring es mayor al radio + el margen siginifica que nos estamos acercando demasiado al borde
                    rospy.loginfo("Cerca del borde (odom), pasando a EVITAR_SALIDA")  #Transicionamos al estado que evita que nos caigamos
                    return 'borde_detectado'  #Return para transicionar

            return 'continuar_busqueda'  #Si no se encuentra nada despues de todos los pasos seguimos en el mismo estado buscando nuestro oponente


class PerseguirOponente(smach.State):  #Este estado ocurre cuando nuestro robot detecta al enemigo y lo empieza a seguir 
    def __init__(self):
        smach.State.__init__(self,outcomes=['oponente_cerca', 'perdido', 'borde_detectado'],input_keys=['laser', 'laser_available', 'odom', 'odom_available'])

    def execute(self, userdata):
        global pub
        rospy.loginfo("Estado: PERSEGUIR_OPONENTE")

        move = Twist()  #Definimos como antes el movimiento como Twist
        rate = rospy.Rate(8)  #Damos una frecuencia para publicar los datos por el topic 

        while not rospy.is_shutdown():  #Aqui esta el publicador de movimiento 
            if userdata.laser_available:  #si el laser esta disponible
                ranges = np.array(userdata.laser.ranges)  #Definimos los rangos del sensor
                n = len(ranges) 

                # Dividir laser en sectores
                p1 = np.nanmin(ranges[:n//4])
                p2 = np.nanmin(ranges[n//4:n//2])
                p3 = np.nanmin(ranges[n//2:3*n//4])
                p4 = np.nanmin(ranges[3*n//4:])
		#Mismo procedimiento que antes
                d = min(p1, p2, p3, p4)
                de = min(p1, p2)
                iz = min(p3, p4)

                # Verificar borde con odometria  (mismo procedimiento que antes)
                if userdata.odom_available:
                    x = userdata.odom.pose.pose.position.x
                    y = userdata.odom.pose.pose.position.y
                    r_ring = 2.0
                    margen = 0.3
                    dist_centro = np.sqrt(x**2 + y**2)
                    if dist_centro > (r_ring - margen):
                        rospy.loginfo("Cerca del borde (odom), pasando a EVITAR_SALIDA")
                        return 'borde_detectado'

                # Si por algun motivo perdemos al oponente y nos da el rango de infinito el sensor, pasamos al estado de nuevo de buscar oponente
                if np.isinf(d):
                    return 'perdido'

                # Girar hacia el oponente segun los sectores
                if iz < de:   #Si la distancia a la derecha es mayor quea la izquierda 
                    move.angular.z = 0.3 #Giramos angularmente a la izquierda  
                elif de < iz:  #Si la distancia a la izquierda es mayor que a la izquierda
                    move.angular.z = -0.3 #Giramos a la derecha
                else:
                    move.angular.z = 0.0  #Cualquier otra cosa nos dejamos rectos

                move.linear.x = 0.2  #A su vez nos estamos moviendo hacia delante para perseguir al robot 
                pub.publish(move)  #Publicamos el movimiento que acabamos de definir 
                rate.sleep() #Lo dejamos en sleep

                # Si persiguiendo al robot enemigo nos acercamos hasta tenerlo a <0.7 m cambiamos de estado
                if d < 0.7:
                    return 'oponente_cerca'  #Tenemos al oponente cerca

        return 'perdido' #Si no ocurre nada de lo anterior significa que perdimos a nuestro oponente, por lo tanto cambiamos al de busqueda de nuevo


class EmpujarOponente(smach.State):  #En este estado nuestro robot detecta a su enemigo muy cerca por lo que acelera para empujarlo y sacarlo del ring
    def __init__(self):
        smach.State.__init__(self,outcomes=['empuje_exitoso', 'oponente_perdido', 'borde_detectado'],input_keys=['laser', 'laser_available', 'odom', 'odom_available'])

    def execute(self, userdata):
        global pub
        rospy.loginfo("Estado: EMPUJAR_OPONENTE")

        move = Twist()  #Definimos el movimiento como antes como Twist
        rate = rospy.Rate(10) #Definimos tambien la frecuencia a la que se van a publicar los mensajes  

        start_time = rospy.Time.now()  #Guardamos el tiempo en el que entramos a este estado
        last_distance = None  #Variable para guardar la ultima distancia al oponente
        bloqueo_detectado = False  #Variable que indica si estamos en un empuje bloqueado (ambos robots empujando sin moverse)

        while not rospy.is_shutdown():  #Aqui tenemos el publicador
            if userdata.laser_available: #Si el laser esta disponible
                ranges = np.array(userdata.laser.ranges)  #Definimos los rangos
                n = len(ranges)

                # Dividir laser en sectores
                p1 = np.nanmin(ranges[:n//4])
                p2 = np.nanmin(ranges[n//4:n//2])
                p3 = np.nanmin(ranges[n//2:3*n//4])
                p4 = np.nanmin(ranges[3*n//4:])
                #Mismo procedimiento que las anteriores veces
                d = min(p1, p2, p3, p4)
                de = min(p1, p2)
                iz = min(p3, p4)

                # Verificar borde con odometria (mismo procedimiento que antes)
                if userdata.odom_available:
                    x = userdata.odom.pose.pose.position.x
                    y = userdata.odom.pose.pose.position.y
                    r_ring = 2.0
                    margen = 0.3
                    dist_centro = np.sqrt(x**2 + y**2)
                    if dist_centro > (r_ring - margen):
                        rospy.loginfo("Cerca del borde (odom), pasando a EVITAR_SALIDA")
                        return 'borde_detectado'

                #Si la distancia es infinita o mayor a 3 perdemos a nuestro oponente de vista, hay que volver a buscarlo
                if np.isinf(d) or d > 3.0:
                    return 'oponente_perdido'

                #Movimiento principal: empujar al oponente
                move.linear.x = 0.5   #Como entramos en este paso significa que el oponente esta muy cerca, aceleramos para tirarlo
                move.angular.z = 0.0
                pub.publish(move) #Publicamos el movimiento

                if d < 0.6:  #Si la distancia es menor a 0.6 mostramos por pantalla que estamos empujando al oponente
                    rospy.loginfo("Empujando al oponente...")

                #A partir dr aqui quiero implementar una estrategia distinta para la pelea
		#Si los dos robots estan de frente empujando infinitamente, si mi robot detecta eso al cabo de un rato, da marcha atras y gira sobre si mismo para empujar al enemigo lateralmente
                if last_distance is None:  #como definimos por primera vez la primera distancia siempre va a ser none 
                    last_distance = d  #Guardamos la primera distancia medida  
                else:
                    time_transcurrido_bloqueo = (rospy.Time.now() - start_time).to_sec()  #Tiempo desde que empezamos a empujar

                    # Si llevamos mas de 5 segundos empujando y la distancia al oponente no cambia mas de 0.1 m
                    if time_transcurrido_bloqueo > 5.0 and abs(d - last_distance) < 0.1:
                        bloqueo_detectado = True  #Aqui marcamos que entramos en esa fase de bloqueo
                        rospy.loginfo("Bloqueo detectado: realizando maniobra evasiva para atacar desde otro angulo") #Lo mostramos por pantalla
                        break  #Salimos del bucle principal para ejecutar la maniobra evasiva

                    last_distance = d  #Actualizamos la ultima distancia medida

                rate.sleep()  #Esperamos un ciclo antes de la siguiente iteracion

        #Si detecta un bloqueo hacemos l maniobra descrita anteriormente 
        if bloqueo_detectado:
            import random  #Voy a importar random para que el robot aleatoriamente decida si girar a la derecha o a la izquierda
            move = Twist()  #Definimos el movimiento como Twist()

            # Retroceder un poco
            move.linear.x = -0.3  #Retrocedemos 
            move.angular.z = 0.0
            for _ in range(20):  #Durante varios ciclos, retrocede lentamente
                pub.publish(move)
                rate.sleep()

            # Girar aleatoriamente a izquierda o derecha
            move.linear.x = 0.0
            move.angular.z = random.choice([-0.7, 0.7])  #Aqui es donde le decimos al robot que decida si girar a la derecha o a la izquierda
            for _ in range(15):  #Hacemos eso durante varios ciclos
                pub.publish(move)  #Publicamos el movimiento
                rate.sleep()

            rospy.loginfo("Maniobra evasiva completada, buscando nuevo angulo de ataque")  #Mostramos por pantalla que la maniobra se completa
            return 'oponente_perdido'  #Volvemos a buscar oponente despues de maniobrar

        # Si llega hasta aqui sin detectar el borde, sin entrar en el bloqueo suponemos que empujo correctamente al oponente
        rospy.loginfo("Empuje exitoso (enemigo desplazado o fuera del ring)")
        
	return 'empuje_exitooso'  #Si empujo al oponete significa que lo tiro del ring, por lo tanto es exitoso



import math
import random

class EvitarSalida(smach.State):  #En este estado entramos cuando detectamos que el robot esta muy cerca del borde, es una maniobra evasiva
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['seguro'],
            input_keys=['laser', 'laser_available', 'odom', 'odom_available'])

    def execute(self, userdata):
        global pub
        rospy.loginfo("Estado: EVITAR_SALIDA")

        move = Twist()  #Definimos el movimiento como Twist
        rate = rospy.Rate(8)  #Definimos una frecuencia de control mas alta

        r_ring = 2.0     #Radio del ring
        margen = 0.3     #Margen de seguridad
        dist_segura = r_ring - (margen + 0.15)  #Distancia considerada segura

        #Si no tenemos odometria disponible, retrocedemos por seguridad
        if not userdata.odom_available:
            rospy.logwarn("Sin odometria disponible, retrocediendo por seguridad")
            move.linear.x = -0.3
            move.angular.z = 0.4
            for _ in range(10):
                pub.publish(move)
                rate.sleep()
            return 'seguro'

        # Obtenemos la posicion actual del robot
        x = userdata.odom.pose.pose.position.x
        y = userdata.odom.pose.pose.position.y

        #En esta parte lo que quiero hacer es obtener la orientacion del robot
	#Para ello utilizamos la odometria del mismo
	#La orientacion la obtenemos a partir de quaternios, por lo tanto nos da una orientacion en x y en y
	#Pero la orientacion que nos interesa es en el eje z (con respecto al ring) por eso calculamos el sin_yaw y el cos_yaw
        orientacion_robot = userdata.odom.pose.pose.orientation
        sin_yaw = 2.0 * (orientacion_robot.w * orientacion_robot.z + orientacion_robot.x * orientacion_robot.y)
        cos_yaw = 1.0 - 2.0 * (orientacion_robot.y * orientacion_robot.y + orientacion_robot.z * orientacion_robot.z)
        angulo_yaw = math.atan2(sin_yaw, cos_yaw)

        # Calculamos el angulo desde el centro hacia la posicion del robot (angulo de salida)
        angulo_salida = math.atan2(y, x)

        # Calculamos la diferencia angular entre donde mira el robot y hacia donde esta el borde
        diferencia_angular = math.atan2(math.sin(angulo_yaw - angulo_salida), math.cos(angulo_yaw - angulo_salida))


        # Si el robot esta mirando hacia el borde (menor de 90 grados), retrocede
        # Si esta mirando hacia el centro (mayor de 90 grados), avanza hacia el centro
        if abs(diferencia_angular) < math.pi / 2:
            rospy.loginfo("Orientado hacia el borde -> retrocediendo")
            move.linear.x = -0.3
        else:
            rospy.loginfo("Orientado hacia el centro -> avanzando")
            move.linear.x = 0.3

        # Anadimos un pequeno giro aleatorio para variar la direccion
        move.angular.z = random.choice([-0.5, 0.5])

        # Ejecutamos el movimiento evasivo durante unos ciclos
        for _ in range(15):
            pub.publish(move)
            rate.sleep()

            # Comprobamos si ya esta lejos del borde
            if userdata.odom_available:
                x = userdata.odom.pose.pose.position.x
                y = userdata.odom.pose.pose.position.y
                dist_centro = np.sqrt(x**2 + y**2)
                print("Distancia actual al centro:", dist_centro, "m")

                if dist_centro < dist_segura:
                    rospy.loginfo("Distancia segura alcanzada, regresando al combate")
                    break

        # Paramos el movimiento antes de salir del estado
        move.linear.x = 0.0
        move.angular.z = 0.0
        pub.publish(move)

        rospy.loginfo("Regresando al combate")
        return 'seguro'


#A partir de aqui es el main del programa, donde ejecutamos la maquina y definimos los estados y las transiciones 

rospy.init_node('sumo_state_machine')  #Iniciamos el nodo del programa
ns = rospy.get_param('-robot_ns', 'robot1')  #Este parametro es para poner el robot 1 en la simulacion del robot
global pub  
pub = rospy.Publisher('/{}/cmd_vel'.format(ns), Twist, queue_size=1)  #Publicador qe publica en el topic de la velocidad

machine = smach.StateMachine(outcomes=['FIN'])  
machine.userdata.laser = LaserScan()
machine.userdata.laser_available = False
machine.userdata.odom = Odometry()
machine.userdata.odom_available = False

scan_sub = rospy.Subscriber('/{}/scan'.format(ns), LaserScan, scan_callback, callback_args=machine.userdata)  #Suscriptor de laser
odom_sub = rospy.Subscriber('/{}/odom'.format(ns), Odometry, odom_callback, callback_args=machine.userdata) #Suscriptor de la odometria

#Definimos los 4 estados de nuestra maquina 

buscar_oponente_state = BuscarOponente()
perseguir_oponente_state = PerseguirOponente()
empujar_oponente_state = EmpujarOponente()
evitar_salida_state = EvitarSalida()

#Aqui defnimios las transiciones de cada estado

with machine:
    smach.StateMachine.add('BUSCAR_OPONENTE', buscar_oponente_state,transitions={'oponente_detectado': 'PERSEGUIR_OPONENTE','continuar_busqueda': 'BUSCAR_OPONENTE','borde_detectado': 'EVITAR_SALIDA'})

    smach.StateMachine.add('PERSEGUIR_OPONENTE', perseguir_oponente_state, transitions={'oponente_cerca': 'EMPUJAR_OPONENTE','perdido': 'BUSCAR_OPONENTE','borde_detectado': 'EVITAR_SALIDA'})

    smach.StateMachine.add('EMPUJAR_OPONENTE', empujar_oponente_state,transitions={'empuje_exitoso': 'BUSCAR_OPONENTE','oponente_perdido': 'BUSCAR_OPONENTE','borde_detectado': 'EVITAR_SALIDA'})

    smach.StateMachine.add('EVITAR_SALIDA', evitar_salida_state,transitions={'seguro': 'BUSCAR_OPONENTE'})

machine.execute()  #Ejecutamos la maquina