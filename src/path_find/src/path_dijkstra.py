#! /usr/bin/env python3

import rospy
import numpy as np
import time
import tf
#from nav_msgs import GetMap
from std_msgs.msg import Int32MultiArray
from tf2_msgs.msg import TFMessage
#cette fonction permet de donner la distance minimale d'un point en le comparant à un autre
def newval(table,pos,prevdist,table_access):
	#on vérifie tout d'abord si le point est hors de la map
	if (pos[0]>(len(table)-1) or pos[0]<0 or pos[1]>(len(table[0])-1) or pos[1]<0):
		return False
	#on vérifie si le point est un mur
	if table_access[pos[0]][pos[1]]==1:
		table[pos[0]][pos[1]]=np.inf
		return False
	#si le point n'avait recu aucune distance précédemment, on lui donne la distance du point dont on vient plus 1
	if table[pos[0]][pos[1]]==-1:
		table[pos[0]][pos[1]]=prevdist+1
		return True
	#si le point avait une distance et quelle est supérieur à la distance du point dont on vient plus 1, on lui donne la nouvelle distance
	if table[pos[0]][pos[1]]>prevdist+1:
		table[pos[0]][pos[1]]=prevdist+1
		return True

def dijkstra_algorithm(table_access, pos):
	#on détermine les dimensions de la map à traiter
	lenx=len(table_access)
	leny=len(table_access[0])
	#on crée une map entièrement faite de -1
	table = np.ones((lenx,leny))
	table= table*(-1)
	#on sauvegarde la position initial du robot
	current_pos=[pos]
	#on met le point d'origine du robot à 0
	table[pos[0]][pos[1]]=0
	#tant qu'il y a des points a explorés, on cherche dans la map
	while (current_pos!=[]):
		newpos=[]
		#on explore à partir de chaque point
		for x in current_pos:
			#on test sur tout les points des points étudier, chaque coté et chaque diagonal en contact avec le point étudier
			#au dessus
			new=newval(table,[x[0]+1,x[1]],table[x[0]][x[1]],table_access)
			#on ajoute si le point a été modifié
			if new==True:
				newpos.append([x[0]+1,x[1]])
			#en dessous
			new=newval(table,[x[0]-1,x[1]],table[x[0]][x[1]],table_access)
			#on ajoute si le point a été modifié
			if new==True:
				newpos.append([x[0]-1,x[1]])
			#a droite
			new=newval(table,[x[0],x[1]+1],table[x[0]][x[1]],table_access)
			#on ajoute si le point a été modifié
			if new==True:
				newpos.append([x[0],x[1]+1])
			#a gauche
			new=newval(table,[x[0],x[1]-1],table[x[0]][x[1]],table_access)
			#on ajoute si le point a été modifié
			if new==True:
				newpos.append([x[0],x[1]-1])
			#au dessus a gauche
			new=newval(table,[x[0]-1,x[1]-1],table[x[0]][x[1]],table_access)
			#on ajoute si le point a été modifié
			if new==True:
				newpos.append([x[0]-1,x[1]-1])
			#au dessus a droite
			new=newval(table,[x[0]-1,x[1]+1],table[x[0]][x[1]],table_access)
			#on ajoute si le point a été modifié
			if new==True:
				newpos.append([x[0]-1,x[1]+1])
			#en dessous a gauche
			new=newval(table,[x[0]+1,x[1]-1],table[x[0]][x[1]],table_access)
			#on ajoute si le point a été modifié
			if new==True:
				newpos.append([x[0]+1,x[1]-1])
			#en dessous a droite
			new=newval(table,[x[0]+1,x[1]+1],table[x[0]][x[1]],table_access)
			#on ajoute si le point a été modifié
			if new==True:
				newpos.append([x[0]+1,x[1]+1])
		current_pos=newpos
	return table
	
def path_find(table,pos):
	path=[pos]
	D=table[pos[0]][pos[1]]
	while (table[pos[0]][pos[1]]!=0):
		'''
		if pos[0]<0:
			if pos[1]<0:
				for k in range(pos[0],pos[0]+1):
					for i in range(pos[1],pos[1]+1):
						if table[k][i]<D:
							D=table[k][i]
							val=[k,i]
				path.append([val[0],val[1]])
				pos=val
			if (pos[1]>len(table[0])):
				for k in range(pos[0],pos[0]+1):
					for i in range(pos[1]-1,pos[1]):
						if table[k][i]<D:
							D=table[k][i]
							val=[k,i]
				path.append([val[0],val[1]])
				pos=val
			else:
				for k in range(pos[0],pos[0]+1):
					for i in range(pos[1]-1,pos[1]+1):
						if table[k][i]<D:
							D=table[k][i]
							val=[k,i]
				path.append([val[0],val[1]])
				pos=val
		if pos[0]>len(table):
			if pos[1]<0:
				for k in range(pos[0]-1,pos[0]):
					for i in range(pos[1],pos[1]+1):
						if table[k][i]<D:
							D=table[k][i]
							val=[k,i]
				path.append([val[0],val[1]])
				pos=val
			if (pos[1]>len(table[0])):
				for k in range(pos[0]-1,pos[0]):
					for i in range(pos[1]-1,pos[1]):
						if table[k][i]<D:
							D=table[k][i]
							val=[k,i]
				path.append([val[0],val[1]])
				pos=val
			else:
				for k in range(pos[0]-1,pos[0]):
					for i in range(pos[1]-1,pos[1]+1):
						if table[k][i]<D:
							D=table[k][i]
							val=[k,i]
				path.append([val[0],val[1]])
				pos=val
		else:
			if pos[1]<0:
				for k in range(pos[0]-1,pos[0]+1):
					for i in range(pos[1],pos[1]+1):
						if table[k][i]<D:
							D=table[k][i]
							val=[k,i]
				path.append([val[0],val[1]])
				pos=val
			if (pos[1]>len(table[0])):
				for k in range(pos[0]-1,pos[0]+1):
					for i in range(pos[1]-1,pos[1]):
						if table[k][i]<D:
							D=table[k][i]
							val=[k,i]
				path.append([val[0],val[1]])
				pos=val
			else:'''
		#on cherche le point le plus proche de la position actuel du robot, en partant de l'objectif
		#on cherche à chaque tour de boucle le point le plus proche et on le définie comme un point de passage du robot
		#la boucle suivante s'executera sur le point de passage nouvellement définie
		#d'abord sur les lignes
		for k in range(pos[0]-1,pos[0]+2):
			#puis sur les colonnes
			for i in range(pos[1]-1,pos[1]+2):
				#on vérifie si ca distance est inférieur à la plus petite déjà identifié
				if table[k][i]<D:
					#si oui, on met a jour la distance, et on sauvegarde le point
					D=table[k][i]
					val=[k,i]
		#une fois étudié tout les points autour du point de passage, on l'ajoute
		path.append([val[0],val[1]])
		#et on met a jour le point
		pos=val
	#à la fin on inverse le sens de la liste afin d'avoir la liste des points de passage dans le bon sens
	path.reverse()
	return(path)

def point_list_cleaner(list_point):
	k=0
	#on parcourt tout les points de la liste
	while (k<(len(list_point)-2)):
		i=1
		#on r"cupère la direction actuelle à prendre
		comp=[b - a for a, b in zip(list_point[k+1], list_point[k])]
		#tant que les points suivant suivent cette direction, on indente i
		while ((comp==[b - a for a, b in zip(list_point[k+i+1], list_point[k+i])]) and (k+i<len(list_point)-2)):
			i+=1
		#si il y a plus d'un point dans la même direction, on les supprime
		if (i>1):
			b=1
			for v in range(1,i):
				if (v%10!=0):
					list_point.pop(k+b)
				else:
					b+=1
		k+=1
	return(list_point)
					

'''
def main():
	table_access=[[1,1,1,1,1,1,1,1],[1,0,0,0,0,0,1,1],[1,0,0,0,0,1,1,1],[1,1,0,0,1,1,0,1],[1,1,0,0,0,0,0,1],[1,1,0,0,0,0,0,1],[1,1,1,1,0,1,0,1],[1,1,1,1,1,1,1,1]]
	table=dijkstra_algorithm(table_access, [6,6])
	print(table)
	path=path_find(table,[1,2])
	print(path)
	clean_path=point_list_cleaner(path)
	print(clean_path)
'''
pub = rospy.Publisher('/path_follow', Int32MultiArray , queue_size=10)


'''
def pos_callback(pos):
	position=pos.transforms
	time.sleep(5)
	posx=position[0].transform.translation.x
	print(posx)
'''	

def dijk_callback(table):
	
	table=dijkstra_algorithm(table_access, position_finale)
	point_de_passage=path_find(table,position_initial)
	point_de_passage_leger=point_list_cleaner(path)
	pub.publish(point_de_passage_leger)

def main():

	# Crée un node qui va récupérer les positions et la map donné par le robot.
	rospy.init_node('dijkstra')
	listener = tf.TransformListener()
	#print("test")
	#sub = rospy.Subscriber('/tf', TFMessage , pos_callback, queue_size=10)
	while 1:
		try:
			(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
			print(trans)
			print(rot)
			time.sleep(5)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
	#print("sub")
	sub1 = rospy.Subscriber('/binary_map_topic', Int32MultiArray , dijk_callback, queue_size=10)
	#print("sub1")
	# spin le node afin de recevoir les messages, et de publier la liste de point de passage.
	rospy.spin()

	# nettoie l'environnement avant de s'arréter.
	rospy.destroy_node()
	rospy.shutdown()

if __name__ == "__main__":
	main()
