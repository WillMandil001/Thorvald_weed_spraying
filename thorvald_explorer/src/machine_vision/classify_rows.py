import numpy as np
import rospy
import pdb

class classify_rows():
	def __init__(self):
		pass

	def classify(self, waypoints, max_dist):
		# INPUT: a list of lists holding image coordinates (row, column)
		rows = [] # initialise an empty list of lists to store the single rows
		row_count = -1 # to keep track of the current class
		next_to_check = [] # to keep track of which other points within the same class need to be examinred before moving on 
		
		while len(waypoints) > 0:
			if len(next_to_check) == 0:
				# Move on to a new starting point if the current thread is fully explored
				rows.append([]) # open a new class (or row)
				next_to_check.append(waypoints[0]) # pick a new point to examine
				row_count += 1
				rows[row_count].append(waypoints[0]) # add the new point to the new row
				del waypoints[0] # remove the point from the remaining

			(r,c) = next_to_check[0]

			distances = [self.euclidean_dist(r,c,waypoints[i][0],waypoints[i][1]) for i in range(len(waypoints))]
			ind = np.argwhere(np.array(distances) < max_dist)
			indeces = np.reshape(ind, [ind.shape[0],])
			if len(indeces) > 0:
				for element in indeces[::-1]:
					rows[row_count].append(waypoints[element])
					next_to_check.append(waypoints[element])
					del waypoints[element]
			del(next_to_check[0])
		return rows

	def euclidean_dist(self,a,b,x,y):
		xdiff = b - y
		ydiff = a - x
		distance = np.sqrt((xdiff**2) + (ydiff**2))
		return distance

if __name__ == '__main__':
	cr = classify_rows()
	rows = cr.classify([[1,2],[2,1],[2,3],[6,6],[7,7]], 2)
	print(rows)