#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

import keyboard

spd = 0.5
trn = 0.3

mov = Twist()

msg="""
Segue a linha de jogos de computador, em tese:
    Movimentos direcionais:
        w: vai para frente
        s: para tras
        a: esquerda
        d: direita
        space_bar: sobe
        ctrl: desce
    Movimentos angulares:
        e: sentido horario (clockwise)
        q: sentido antihorario (counterclockwise)
    Teclas de controle:
        Enter: take_off
        Backspace: land

controle de velocidade ainda n implementado
sinta-se a vontade para fazer alteracoes
ele detecta movimentos complexos(ir para a diagonal direita: w e d simultaneamente)
"""

takeoff_topic = rospy.Publisher("/tello/takeoff", Empty, queue_size=1)
land_topic = rospy.Publisher("/tello/land", Empty, queue_size=1)
pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size = 1)

rospy.init_node("controle tello provisorio")

def key_pressed(e):
    mov.linear.x = 0
    mov.linear.y = 0
    mov.linear.z = 0
    mov.angular.z = 0

    if keyboard.is_pressed('Backspace'):
        land_topic.publish(Empty())
    else if keyboard.is_pressed('Enter'):
        takeoff_topic.publish(Empty())
    else:
        mov.linear.x = spd*keyboard.is_pressed('w') - spd*keyboard.is_pressed('s')
        mov.linear.y = spd*keyboard.is_pressed('a') - spd*keyboard.is_pressed('d')
        mov.linear.z = spd*keyboard.is_pressed(' ') - spd*keyboard.is_pressed('ctrl')
        mov.angular.z = trn*keyboard.is_pressed('q') - trn*keyboard.is_pressed('e')
    pub.publish(mov)


keyboard.hook(key_pressed)
keyboard.wait()
