#!/usr/bin/env python3

"""
Nombre del programa: frm_lab3_ev3_delegate.py
Autor: Andres Felipe Zuleta Romero
Curso: Fundamentos de robotica movil
Departamento de Ingenieria Mecanica y Mecatronica
Universidad Nacional de Colombia - Sede Bogota
2024-1S.

"""

import mqtt_remote_method_calls as com
from ev3dev2.button import Button
from ev3dev2.motor import OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, MediumMotor
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
from ev3dev2.wheel import EV3EducationSetTire
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, lego

from frm_lab3_move_ev3 import MoveTankMission1
import time
import sys
import math

#Creacion del Delegado 
class EV3RobotDelegate(object):

    """ Los metodos de esta clase seran los que procesen los mensajes MQTT recibidos
        los atributos seran elementos de control necesarios en el procesamiento """
    
    def __init__(self):
        #Se declara el cliente mqtt como None
        self.mqtt_client = None

        #Elementos para mensajes a enviar
        self.msgtype = None
        self.msglist = None

        # Definir el tipo de llanta usado
        self.llanta = EV3EducationSetTire()
        STUD_MM = 8

        #Se inicializan los motores
        self.tank = MoveTankMission1(OUTPUT_B, OUTPUT_C,EV3EducationSetTire, 15 * STUD_MM)
        self.medium_motor = MediumMotor(OUTPUT_D)

        #Se inicializan los sensores
        self.ultrasonic_sensor = lego.UltrasonicSensor(INPUT_3)
        self.gyro_sensor = lego.GyroSensor(INPUT_2)
        self.touch_sensor = lego.TouchSensor(INPUT_1)

        #Se asigna el giroscopio al tank
        self.tank.ultrasonic_s = self.ultrasonic_sensor
        self.tank.uts_motor = self.medium_motor
        self.tank.touch_sensor = self.touch_sensor
        self.tank.gyro = self.gyro_sensor
        self.gyro_sensor.calibrate()
        self.tank.odometry_start()

        #Sonido
        self.sound = Sound()

        #Tiempo de funcionamiento 
        self.seconds = 0.4
        self.speed = 30

    def set_mqtt_client(self, mqtt_client):
        #Funcion para declara el cliente mqtt que se usara
        self.mqtt_client = mqtt_client
    def reset_gyro_sensor(self):
        self.gyro_sensor.reset()
    def move_medium_motor(self, speed, angle):
        self.medium_motor.on_to_position(speed=speed,position=angle)
    def stop(self):
        #Funcion para parar cualquier movimiento
        self.tank.stop()
    def quit(self):
        #Funcion para salir de la rutina parando todos los movimientos
        self.tank.stop()
        print("Quit to the routing")
        self.running = False
        Leds().set_color("LEFT", "BLACK")
        Leds().set_color("RIGHT", "BLACK")
        sys.exit()
        broke
    def shutdown(self):
        #Funcion para apagar la rutina si el cliente MQTT se cierra
        self.sound.speak('Goodbye')
        self.running = False
        self.tank.odometry_stop()
        Leds().set_color("LEFT", "BLACK")
        Leds().set_color("RIGHT", "BLACK")
        sys.exit()

    # Mision 1 del laboratorio
    def move_to_goal(self, goal_x, goal_y, speed, safe_front_distance, distance_to_start_goal_line_precision, leave_point_to_hit_point_diff, dist_thresh_wf):
        self.tank.run_mission(speed,goal_x, goal_y,safe_front_distance,distance_to_start_goal_line_precision,leave_point_to_hit_point_diff,dist_thresh_wf)
    
    # Mision 2 del laboratorio
    def run_mission_2(self, speed_lineal,speed_angular,safe_distance):
        self.tank.run_mission_2(speed_lineal,speed_angular,safe_distance)

    def turn_to_angle(self, speed, angle):
        self.tank.turn_to_angle(speed,angle,use_gyro=True)

    def move_circle(self, radius_mm, distance_mm, speed):
        self.tank.on_arc_left(speed,radius_mm,distance_mm)

    def loop_forever(self):
        #Funcion de funcionamiento hasta que se oprime el boton backspace del robot
        button = Button()
        while not button.backspace:
            time.sleep(0.25)
            #self.sensing()

        if self.mqtt_client:
            self.mqtt_client.close()
        self.shutdown()
        sys.exit()

    def sensing(self):
        distance_cm = self.ultrasonic_sensor.distance_centimeters
        gyro_value = self.gyro_sensor.angle
        touch_state = self.touch_sensor.is_pressed
        medium_motor_angle = self.medium_motor.position

        msgtype = "update_ev3_readings"
        msglist = [distance_cm, gyro_value, touch_state, medium_motor_angle]

        self.mqtt_client.send_message(msgtype,msglist)

        """Impresion de accion realizada, topico en donde se publica
        el mensaje y el mensaje enviado"""

        print(self.mqtt_client.publish_topic_name)
        print(msgtype, msglist)

def main():

    #Impresion de que se ha iniciado el robot y se ponen los leds en rojo
    print("Start robot")
    Leds().set_color("LEFT", "RED")
    Leds().set_color("RIGHT", "RED")

    #Se crea el delegado de recepcion de mensajes MQTT y se ponen los leds en amber
    delegate = EV3RobotDelegate()
    mqtt_client = com.MqttClient(delegate)
    Leds().set_color("LEFT", "AMBER")
    Leds().set_color("RIGHT", "AMBER")

    #Se declara el cliente mqtt como el cliente que usara el delegado
    delegate.set_mqtt_client(mqtt_client)

    #Se conecta al topico para sucricon y publicacion con el PC y se poenen los leds en verde
    mqtt_client.connect_to_pc()
    Leds().set_color("LEFT", "GREEN")
    Leds().set_color("RIGHT", "GREEN")

    #Se crea el bulce infinito de funcionamiento
    delegate.loop_forever()
    print("Shutdown complete")

#Se inicial el main
main()
