#! /usr/bin/env python3

import numpy as np
import rospy
import roslib
import math
from std_msgs.msg import Int32MultiArray
from tf2_msgs.msg import TFMessage
import tf
from geometry_msgs.msg  import Twist
from std_msgs.msg import Float64MultiArray

rospy.init_node('path_follow')
origin_x = 1.0
origin_y =1.0
corner_top_right_x =1.0
corner_top_right_y =1.0
corner_bottom_left_x =1.0
corner_bottom_left_y =1.0
leny=1.0
lenx=1.0
verif=1

listener = tf.TransformListener()
#commande de ralliement de point
def loi_commande(pos_rob,pos_obj, theta):
	k1=1
	k2=1
	l1=30 #point théorique placé 30 cm devant le robot
	#on applique un ralliement de point de passage
	v1=k1*(pos_rob[0]-pos_obj[0])
	v2=k2*(pos_rob[1]-pos_obj[1])
	v=np.array(([v1],[v2]))
	tab=np.array(([np.cos(theta),-l1*np.sin(theta)],[np.sin(theta),l1*np.cos(theta)]))
	invtab=np.linalg.inv(tab)
	u=np.dot(invtab,v)
	u[1]=-u[1]
	#si la vitesse devient trop faible, on passe au point suivant
	if (u[0]>1 or u[0]<1):
		verif=0;
	else:
		verif=1;
	return (u)

	
'''
def main():

	[u1,u2]=loi_commande([20,25],[25,25],30)
	print("u1",u1,"u2",u2)
'''
 
def euler_from_quaternion(x, y, z, w):
        """
        converti les quaternions en angles d'euler (roll, pitch, yaw)
        x, y et z sont en radians
        roll autour de x en sans anti horraire
        pitch autour de y (counterclockwise)
        yaw autour de z (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # en radians

pub = rospy.Publisher('/cmd_vel', Twist , queue_size=10)

def len_callback(table_access):
	global leny
	global lenx
	lenx=table_access.layout.dim[1].size #len(table_access[0])
	leny=table_access.layout.dim[0].size #len(table_access[0])


def pos_callback(pos):
	cmd_vel= Twist()
	global origin_x
	global origin_y
	global corner_top_right_x
	global corner_top_right_y
	global corner_bottom_left_x
	global corner_bottom_left_y
	global lenx
	global leny
	#pour chaque point
	print(len(pos.data)/2)
	length=int(len(pos.data)/2)
	print(length)
	for k in range(length):
		#on récupère la position du robot
		(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
		#print(trans)
		#print(rot)
		ratio_x=lenx/(corner_top_right_x-origin_x)
		ratio_y=leny/(corner_bottom_left_y-origin_y)
		#on la convertie en angle d'euler
		newrot=euler_from_quaternion(rot[0],rot[1],rot[2],rot[3])
		position=[trans[0],trans[1]]
		position=[(position[0]-origin_x)*ratio_x,leny-((position[1]-origin_y)*ratio_y)]
		#si la vitesse est trop faible, on passe au point suivant
		if (verif==0):
			k=k-1
		#sinon on garde le point précédent
		u=loi_commande([trans[0],trans[1]],[pos.data[2*k],pos.data[2*k+1]], newrot[2])
		print(u)
		cmd_vel.linear.x=u[0]
		cmd_vel.linear.y=0
		cmd_vel.linear.z=0
		cmd_vel.angular.x=0
		cmd_vel.angular.y=0
		cmd_vel.angular.z=u[1]
		
		pub.publish(cmd_vel)
	cmd_vel.linear.x=0
	cmd_vel.linear.y=0
	cmd_vel.linear.z=0
	cmd_vel.angular.x=0
	cmd_vel.angular.y=0
	cmd_vel.angular.z=0
	pub.publish(cmd_vel)
def corners_callback(msg):

	global origin_x
	global origin_y
	global corner_top_right_x
	global corner_top_right_y
	global corner_bottom_left_x
	global corner_bottom_left_y
	origin_x = msg.data[0]  # Origine de la carte (Position en coordonnées de la carte)
	origin_y = msg.data[1]  # Origine de la carte (Position en coordonnées de la carte)
	corner_top_right_x=msg.data[2]
	corner_top_right_y=msg.data[3]
	corner_bottom_left_x=msg.data[4]
	corner_bottom_left_y=msg.data[5]
	

def main():
	# Crée un node qui va récupérer les positions et la map donné par le robot.
	listener = tf.TransformListener()
	
	sub = rospy.Subscriber('/path_follow',Int32MultiArray , pos_callback, queue_size=10)
	sub3 = rospy.Subscriber('/map_corners', Float64MultiArray , corners_callback)
	sub1 = rospy.Subscriber('/binary_map_topic', Int32MultiArray , len_callback, queue_size=10)
	# spin le node afin de recevoir les messages, et de publier la liste de point de passage.
	rospy.spin()

	# nettoie l'environnement avant de s'arréter.
	rospy.destroy_node()
	rospy.shutdown()

if __name__ == "__main__":
	main()
