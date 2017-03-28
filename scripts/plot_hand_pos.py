#!/usr/bin/env python
'''
==============
3D Hand position scatterplot
==============

Test .
'''

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
	
def plot_hand_pos(hand_pos):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	for row in hand_pos:
		x=row[0]
		y=row[1]
		z=row[2]
		ax.scatter(x, y, z, c='r', marker='o')
	
	ax.set_xlabel('X [mm]')
	ax.set_ylabel('Y [mm]')
	ax.set_zlabel('Z [mm]')
	plt.suptitle('Hand position')
	
	fig.savefig('hand_pos.pdf')
	fig.savefig('hand_pos.svg')

	#plt.show()
	
	return;


hand_pos = np.array([
					 [1,2,3],
					 [-6,8,-4],
					 [5,7,-12]
					])

plot_hand_pos(hand_pos)