#! /usr/bin/env python3

import numpy as np
import rospy
import roslib
import math

from geometry_msgs.msg  import Twist
#commande de ralliement de point
def loi_commande(pos_rob,pos_obj, theta):
	k1=1
	k2=1
	l1=30 #point théorique placé 30 cm devant le robot
	v1=k1*(pos_rob[0]-pos_obj[0])
	v2=k2*(pos_rob[1]-pos_obj[1])
	v=np.array(([v1],[v2]))
	tab=np.array(([np.cos(theta),-l1*np.sin(theta)],[np.sin(theta),l1*np.cos(theta)]))
	invtab=np.linalg.inv(tab)
	u=np.dot(invtab,v)
	return (u)

	
'''
def main():

	[u1,u2]=loi_commande([20,25],[25,25],30)
	print("u1",u1,"u2",u2)
'''
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
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
     
        return roll_x, pitch_y, yaw_z # in radians

pub = rospy.Publisher('/consigne', Int32MultiArray , queue_size=10)

def pos_callback(pos):
	for k in range(len(pos)):
		(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
		print(trans)
		print(rot)
		newrot=euler_from_quaternion(rot[0],rot[1],rot[2],rot[3])
		if (verif=1):
			u=loi_commande([trans[0],trans[1]],pos[k], newrot[2])
			pub.publish(u)
		else:
			k=k-1
		

def main()
	# Crée un node qui va récupérer les positions et la map donné par le robot.
	listener = tf.TransformListener()
	
	sub = rospy.Subscriber('/path_follow',Int32MultiArray , path_callback, queue_size=10)
	
	# spin le node afin de recevoir les messages, et de publier la liste de point de passage.
	rospy.spin()

	# nettoie l'environnement avant de s'arréter.
	rospy.destroy_node()
	rospy.shutdown()

if __name__ == "__main__":
	main()
