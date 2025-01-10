#! /usr/bin/env python3

import numpy as np
import rospy
import roslib

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
	

def main():

	[u1,u2]=loi_commande([20,25],[25,25],30)
	print("u1",u1,"u2",u2)
	
if __name__ == "__main__":
	main()
