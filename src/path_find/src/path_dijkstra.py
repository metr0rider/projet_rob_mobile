#! /usr/bin/env python3

import rospy
import numpy as np
import time
import tf
from geometry_msgs.msg import PointStamped, Pose, Point, Quaternion
#from nav_msgs import GetMap
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid


rospy.init_node('dijkstra')
resolution =1
# Initialisation de la position (x, y, z) de l'origine
origin_x = 0.0  # Par exemple, à l'origine du système de coordonnées
origin_y = 0.0
corner_top_right_x = 1.0
corner_top_right_y = 0.0
corner_bottom_left_x = 0.0
corner_bottom_left_y = -1.0
width_map= 1.0
height_map= 1.0
lenx=1
leny=1
ratio_x=1.0
ratio_y=1.0
def map_callback(msg):
	global resolution
	global width_map
	global height_map
	resolution = msg.info.resolution  # Résolution en mètres par pixel
	width_map = msg.info.width
	height_map= msg.info.height

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


point = PointStamped()
trans=[0,0,0]
rot=[0,0,0,0]
listener = tf.TransformListener()
point_de_passage_leger=[]
time_to_publish=False
position_initial=[0.0,0.0]
position_finale=[0.0,0.0]

#cette fonction permet de donner la distance minimale d'un point en le comparant à un autre
def newval(table,pos,prevdist,table_access):
	stride_x= table_access.layout.dim[1].stride
	#on vérifie tout d'abord si le point est hors de la map
	if (pos[0]>(len(table)-1) or pos[0]<0 or pos[1]>(len(table[0])-1) or pos[1]<0):
		return False
	#on vérifie si le point est un mur
	if table_access.data[pos[1]*stride_x+pos[0]]==1:
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
	global lenx
	global leny
	#on crée une map entièrement faite de -1
	table = np.ones((lenx,leny))
	table= table*(-1)
	#on sauvegarde la position initial du robot
	posx=int(pos[0]*lenx)
	posy=int(pos[1]*leny)
	print(posx,posy)
	current_pos=[[posx,posy]]
	#on met le point d'origine du robot à 0
	table[posx][posy]=0
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
	
def path_find(table,pos,table_access):
	stride_x= table_access.layout.dim[1].stride
	print(stride_x)
	global lenx
	global leny
	posx=int(pos[0]*lenx)
	posy=int(pos[1]*leny)
	print(posx,posy)
	begin_pos=[[posx,posy]]
	path=[[posx,posy]]
	D=table[posx][posy]
	val=[posx,posy]
	visited_positions = set()
	while (table[posx][posy]!=0):
		if (posx, posy) in visited_positions:
			print(table_access.data[posy*stride_x+posx])
			print("Loop detected, stopping...")
			break
		visited_positions.add((posx, posy))
		#on cherche le point le plus proche de la position actuel du robot, en partant de l'objectif
		#on cherche à chaque tour de boucle le point le plus proche et on le définie comme un point de passage du robot
		#la boucle suivante s'executera sur le point de passage nouvellement définie
		#d'abord sur les lignes
		for k in range(max(0,posx-1),min(len(table),posx+2)):
			#puis sur les colonnes
			for i in range(max(0,posy-1),min(len(table[0]),posy+2)):
				#on vérifie si sa distance est inférieur à la plus petite déjà identifié
				if table[k][i]<D:
					#si oui, on met a jour la distance, et on sauvegarde le point
					D=table[k][i]
					val=[k,i]
		#une fois étudié tout les points autour du point de passage, on l'ajoute
		path.append([val[0],val[1]])
		posx=val[0]
		posy=val[1]
		#et on met a jour le point
		pos=val
	#à la fin on inverse le sens de la liste afin d'avoir la liste des points de passage dans le bon sens
	#path.reverse()
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
# on initialise le publisher de la liste de point en globale
pub = rospy.Publisher('/path_follow', Int32MultiArray , queue_size=10)


'''
def pos_callback(pos):
	position=pos.transforms
	time.sleep(5)
	posx=position[0].transform.translation.x
	print(posx)
'''

def dijk_callback(table_access):
	#si un point d'arriver est défini, on publie
	point_list = Int32MultiArray()
	global time_to_publish
	global position_finale
	global position_initial
	global origin_x
	global origin_y
	global corner_top_right_x
	global corner_top_right_y
	global corner_bottom_left_x
	global corner_bottom_left_y
	global lenx
	global leny
	global ratio_x
	global ratio_y
	#on détermine les dimensions de la map à traiter
	lenx=table_access.layout.dim[1].size  #len(table_access)
	leny=table_access.layout.dim[0].size #len(table_access[0])
	print(lenx,leny)
	tab=[]
	if (time_to_publish==True):
		ratio_x=lenx/(corner_top_right_x-origin_x)
		ratio_y=leny/(corner_bottom_left_y-origin_y)
		position_finale=[(position_finale[0]-origin_x)*ratio_x,leny-((position_finale[1]-origin_y)*ratio_y)]
		position_initial=[(position_initial[0]-origin_x)*ratio_x,leny-((position_initial[1]-origin_y)*ratio_y)]
		#on trouve la distance de chaque point de la table par rapport a l'arrivé
		table=dijkstra_algorithm(table_access, position_finale)
		#print(table)
		#with open("tableau.txt", "w") as fichier:
    		#	for ligne in table:
        	#		fichier.write(" ".join(map(str, ligne)) + "\n")
		#print("Le tableau a été sauvegardé dans tableau.txt.")
		#input("Appuyez sur Entrée pour continuer...")
		#on trouve le chemin le plus court vers l'arrivé
		point_de_passage=path_find(table,position_initial,table_access)
		#on allège la liste des points inutiles
		print(point_de_passage)
		point_de_passage_leger=point_list_cleaner(point_de_passage)
		#on publie la liste de point
		for k in range (len(point_de_passage_leger)):
			for i in range (len(point_de_passage_leger[0])):
				tab.append(point_de_passage_leger[k][i])
		print(tab)
		point_list.data=tab
		pub.publish(point_list)
		#on re désactive la publication
	time_to_publish=False
	
def pos_callback(pos):
	global time_to_publish
	global point
	global position_finale
	global trans
	global rot
	global position_initial
	global resolution
	#print(origin_x)
	#print(origin_y)
	#print(width_map)
	#print(height_map)
	#on récupère les différentes info de la position
	point.header.stamp = rospy.get_time
	point.header.frame_id = "/map"
	point.point.x = pos.point.x
	point.point.y = pos.point.y
	point.point.z = pos.point.z
        
	#on affiche la position à atteindre
	rospy.loginfo("coordinates:x=%f y=%f" %(point.point.x,point.point.y))
	#on sauvegarde cette position
	position_finale=[point.point.x,point.point.y]
	#on prend la position actuel du robot
	print(position_finale)
	(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
	#print(trans)
	#print(rot)
	#on sauvegarde cette position
	position_initial=[trans[0],trans[1]]
	print(position_initial)
	#on active le path finding
	time_to_publish=True
	

def main():
	# Crée un node qui va récupérer les positions et la map donné par le robot.
	listener = tf.TransformListener()
	#print("test")
	#sub = rospy.Subscriber('/tf', TFMessage , pos_callback, queue_size=10)
	#print("sub")
	sub0 = rospy.Subscriber('/map', OccupancyGrid, map_callback)
	sub1 = rospy.Subscriber('/binary_map_topic', Int32MultiArray , dijk_callback, queue_size=10)
	sub2 = rospy.Subscriber('/clicked_point', PointStamped , pos_callback)
	sub3 = rospy.Subscriber('/map_corners', Float64MultiArray , corners_callback)
	#print("sub1")
	# spin le node afin de recevoir les messages, et de publier la liste de point de passage.
	rospy.spin()

	# nettoie l'environnement avant de s'arréter.
	rospy.destroy_node()
	rospy.shutdown()

if __name__ == "__main__":
	main()
