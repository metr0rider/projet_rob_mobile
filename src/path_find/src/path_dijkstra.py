#! /usr/bin/env python3

import rospy
import numpy as np

def newval(table,pos,prevdist,table_access):
#	print("la position est:")
#	print(pos[0])
#	print(pos[1])
#	print(pos[0]>(len(table)) or pos[0]<0 or pos[1]>(len(table[0])) or pos[1]<0)
	if (pos[0]>(len(table)-1) or pos[0]<0 or pos[1]>(len(table[0])-1) or pos[1]<0):
		return False
#	print("la valeur est")
#	print(table[pos[0]][pos[1]])
	if table_access[pos[0]][pos[1]]==1:
		table[pos[0]][pos[1]]=np.inf
#		print("la nouvelle valeur est")
#		print(table[pos[0]][pos[1]])
		return False
	if table[pos[0]][pos[1]]==-1:
		table[pos[0]][pos[1]]=prevdist+1
#		print("la nouvelle valeur est")
#		print(table[pos[0]][pos[1]])
		return True
	if table[pos[0]][pos[1]]>prevdist+1:
		table[pos[0]][pos[1]]=prevdist+1
#		print("la nouvelle valeur est")
#		print(table[pos[0]][pos[1]])
		return True

def dijkstra_algorithm(table_access, pos):
	lenx=len(table_access)
	leny=len(table_access[0])
	table = np.ones((lenx,leny))
	table= table*(-1)
	current_pos=[pos]
	table[pos[0]][pos[1]]=0
	while (-1 in table):
#		print(current_pos)
		newpos=[]
		for x in current_pos:
			new=newval(table,[x[0]+1,x[1]],table[x[0]][x[1]],table_access)
			if new==True:
				newpos.append([x[0]+1,x[1]])
			new=newval(table,[x[0]-1,x[1]],table[x[0]][x[1]],table_access)
			if new==True:
				newpos.append([x[0]-1,x[1]])
			new=newval(table,[x[0],x[1]+1],table[x[0]][x[1]],table_access)
			if new==True:
				newpos.append([x[0],x[1]+1])
			new=newval(table,[x[0],x[1]-1],table[x[0]][x[1]],table_access)
			if new==True:
				newpos.append([x[0],x[1]-1])
		current_pos=newpos
	return table
	
def main():
	table_access=[[0,0,0,0,1],[0,0,0,1,1],[0,0,1,1,0],[0,0,0,0,0],[0,0,0,0,0]]
	table=dijkstra_algorithm(table_access, [2,1])
	print(table)
	
if __name__ == "__main__":
    main()
