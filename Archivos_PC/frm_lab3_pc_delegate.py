
#Impotación de las librerias necesarias
from mqtt_remote_method_calls import MqttClient
import time
import tty
import sys
import termios
import math

#Creación de la clase personalizada para la recepción de mensajes MQTT
class PCDelegate(object):

    """ Los metodos de esta clase seran los que procesen los mensajes MQTT recibidos
        los atributos seran elementos de control necesarios en el procesamiento """

    def __init__(self):

        #Se declara el cliente mqtt como None
        self.mqtt_client = None

        #Elementos para mensajes a enviar
        self.msgtype = None
        self.msglist = None

        #Variable de lectura del sensor ultrasonido
        self.ultrasonic_distance = 255
        #Variable de lectura del girosensor
        self.robot_angle = None
        #Variable de estado del sensor touch
        self.is_touch_sensor_pressed = None
        #Variavle de posicion del sensor del motor mediano
        self.medium_motor_position = None

    def update_ev3_readings(self, distance_cm, gyro_value, touch_state, medium_motor_angle):

        self.ultrasonic_distance = distance_cm
        self.robot_angle = gyro_value
        self.is_touch_sensor_pressed = touch_state
        self.medium_motor_position = medium_motor_angle

        print("Distancia: "+str(distance_cm)+" cm")
        print("Valor giroscopio: "+str(gyro_value)+" deg")
        print("Sensor Touch presionado: "+str(touch_state))
        print("Angulo motor: "+str(medium_motor_angle)+" deg")
    
    def set_mqtt_client(self, mqtt_client):
        #Funcion para declara el cliente mqtt que se usara
        self.mqtt_client = mqtt_client

    def reset_gyro_sensor(self):
        msgtype = "reset_gyro_sensor"
        mqtt_client.send_message(msgtype)
        self.robot_angle = 0

    def move_medium_motor(self, speed, angle):
        #Declaración del tipo de mensaje y lista de parametros
        msgtype = "move_medium_motor"
        msglist = [speed, angle]
        
        #Envio de mensaje tipo "drive" y mensaje con lista de parametros
        mqtt_client.send_message(msgtype, msglist)

    def move_to_goal(self, goal_x, goal_y, speed, safe_front_distance, distance_to_start_goal_line_precision, leave_point_to_hit_point_diff, dist_thresh_wf):
        #Declaración del tipo de mensaje y lista de parametros
        msgtype = "move_to_goal"
        msglist = [goal_x, goal_y, speed, safe_front_distance, distance_to_start_goal_line_precision, leave_point_to_hit_point_diff, dist_thresh_wf]
        
        #Envio de mensaje tipo "drive" y mensaje con lista de parametros
        mqtt_client.send_message(msgtype, msglist)
    
    def run_mission_2(self, speed_lineal,speed_angular,safe_distance):
        #Declaración del tipo de mensaje y lista de parametros
        msgtype = "run_mission_2"
        msglist = [speed_lineal, speed_angular, safe_distance]
        
        #Envio de mensaje tipo "drive" y mensaje con lista de parametros
        mqtt_client.send_message(msgtype, msglist)

    def turn_to_angle(self, speed, angle):
        #Declaración del tipo de mensaje y lista de parametros
        msgtype = "turn_to_angle"
        msglist = [speed, angle]
        
        #Envio de mensaje tipo "drive" y mensaje con lista de parametros
        mqtt_client.send_message(msgtype, msglist)
    
    def move_circle(self, radius_mm, distance_mm, speed):
        #Declaración del tipo de mensaje y lista de parametros
        msgtype = "move_circle"
        msglist = [radius_mm, distance_mm, speed]
        
        #Envio de mensaje tipo "drive" y mensaje con lista de parametros
        mqtt_client.send_message(msgtype, msglist)

    def stop_robot(self):
        #Declaración del tipo de mensaje sin lista de parametros
        msgtype = "stop"

        #Envio de mensaje
        mqtt_client.send_message(msgtype)

        print("STOPPING ROBOT")

if __name__ == '__main__':

    #Cración de delegado para mensajes MQTT y cliente MQTT
    delegate = PCDelegate()
    mqtt_client = MqttClient(delegate)

    #Creación de suscripcion a topico MQTT LEGOEV301/msgPC y publicacion en topico  LegoEV301/msgLegoEv3
    mqtt_client.connect_to_ev3()
    
    #Se declara el cliente mqtt como el cliente que usara el delegado
    delegate.set_mqtt_client(mqtt_client)


    time.sleep(1)
    #delegate.move_to_goal(goal_x=2500,goal_y=-1000,speed=10,safe_front_distance=18,distance_to_start_goal_line_precision=5,leave_point_to_hit_point_diff=60,dist_thresh_wf=2)
    delegate.run_mission_2(speed_lineal=10,speed_angular=10,safe_distance=17)
    #delegate.turn_to_angle(20,-45)

    #Cerrado del cliente MQTT
    mqtt_client.close()
