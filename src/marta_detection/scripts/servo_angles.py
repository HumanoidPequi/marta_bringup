#!/usr/bin/env python3
#todo script é um nó diferente
#esse nó pega as coordenadas da bola que foram publicadas pelo nó "ball_tracking" e publica os angulos para o servo
import rospy
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Float32, Float64
import rospkg
import yaml
import numpy as np
import math as mt
import os
import time

pub_angles = rospy.Publisher('/marta/arm_l_head/command', Int16MultiArray, queue_size=1)

pub_angles_pitch = rospy.Publisher('/marta/head_pan_position/command', Float64, queue_size=1)
pub_angles_yaw = rospy.Publisher('/marta/head_tilt_position/command', Float64, queue_size=1)

angles = Int16MultiArray()

package_name = 'marta_detection'
rospack = rospkg.RosPack()
package_path = rospack.get_path(package_name)

last_pitch = 0
last_yaw = 0

with open(os.path.join(package_path, 'params/params.yaml')) as f:
    cam_params = yaml.load(f, Loader=yaml.FullLoader)
    intrinsic = np.array(cam_params['mtx'])

fx = intrinsic[0,0]
fy = intrinsic[1,1]

cx = 320
cy = 240

theta_z = 0   #< >
theta_y = 0 #cima e baixo ^v

h_cam = 30 #cm originalmente 68
ball_distance = Float32()

pub_distance = rospy.Publisher("ball_distance", Float32, queue_size = 1)

def angles_callback(msg):#toda vez wue eu receber a posicao da bola eu vou executar o callback
    #esse calback calcula os 2 angulos da cabeça e publica
    global theta_y, theta_z
    
    if msg.data[0] !=1000 and msg.data[1] !=1000:
        
        v = msg.data[0]
        u = msg.data[1]
                                                
        if abs(v - cx) > 25 or abs(u - cy) > 20:
            #rospy.loginfo("entrei no segundo if")
            x = -(v-cx)
            y = -(u-cy)
            #if (x >=15) and (y>=15):
            theta_z = theta_z + float(np.arctan2(x,fx)*180/mt.pi) 
            theta_y = theta_y + float(np.arctan2(y,fy)*180/mt.pi) 

            rospy.loginfo("theta_z: %s, theta_y: %s", theta_z, theta_y)

            #if(theta_y<-70):
            #    theta_y = -70
            #if theta_y > -20:
            #    theta_y = -20
            #if theta_z < -50:
            #    theta_z = -50 
            #if theta_z > 50:
            #    theta_z = 50

            angles.data = [theta_z,theta_y,0,0,0]
        
    else:
        #rospy.loginfo("não to entrando em porra nenhuma")
        theta_z = 0
        theta_y = -50
        angles.data = [theta_z,theta_y,0,0,0]
        
    #rospy.loginfo('the angles are %i, %i', theta_y,theta_z)
    #pub_angles.publish(angles)

    publish_angles()

    alpha = 90 - angles.data[1]
    #print(mt.tan(alpha))
    abacate = mt.radians(alpha)
    #print(abacate)
    #rospy.loginfo("este é alpha %s", alpha)
    ball_distance.data = mt.tan(abacate) * h_cam
    #print(mt.tan(abacate))
    #rospy.loginfo("este é ball_distance.data %s", ball_distance.data)
    pub_distance.publish(ball_distance)

def publish_angles():
    global last_pitch, last_yaw
    N = 100

    rospy.loginfo(f"pitch: {mt.radians(angles.data[1])*180/mt.pi}, yaw: {mt.radians(angles.data[0])*180/mt.pi}")

    dif_pitch = (mt.radians(angles.data[1]) - last_pitch)/N
    dif_yaw = (mt.radians(angles.data[0]) - last_yaw)/N

    for i in range(N):
        pub_angles_pitch.publish(last_pitch + dif_pitch*i)
        pub_angles_yaw.publish(last_yaw + dif_yaw*i)
        rospy.loginfo(f"pitch: {last_pitch + dif_pitch*i}, yaw: {last_yaw + dif_yaw*i}")
        time.sleep(10 / N)

    time.sleep(10)
    last_pitch = mt.radians(angles.data[1])
    last_yaw = mt.radians(angles.data[0])

def angles_sub():
    rospy.Subscriber('/ball_pose', Float32MultiArray, angles_callback, queue_size=1)
    rospy.spin()

if __name__=='__main__':
    rospy.init_node('angles_publisher')

    angles_sub()