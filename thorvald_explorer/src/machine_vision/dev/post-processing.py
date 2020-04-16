import rospy
import numpy as np
import matplotlib.pyplot as plt
import pickle
import math
import cv2

class post_processing():
	def __init__(self):
		with open('Evaluation/toponav.waypoints', 'rb') as wp_file:
			self.all_waypoints = pickle.load(wp_file)
		self.max_angle = 0.025
		self.max_deviation = 20

	def drop_redundant_wp(self, wp):
		sparse_wp = []
		row_count = -1
		for row in wp:
			row_count += 1
			sparse_wp.append([])
			anchor_point = []
			for point in row:
				if not anchor_point:
					previous_angle = []
					anchor_point = tuple(point)
					sparse_wp[row_count].append(point)
				else:
					dx = point[0]-anchor_point[0]
					dy = point[1]-anchor_point[1]
					angle = math.atan2(dy, dx)
					if angle < 0:
						sign = -1
					else:
						sign = 1
					angle = abs(angle)
					while angle > (math.pi/2):
						angle -= (math.pi/2)

					if not previous_angle:
						# Previous angle is always from the current anchor point
						# to the immediate next point
						previous_angle = sign * angle
						prev_point = tuple(point)
					else:
						length = math.sqrt(dx**2 + dy**2)
						delta_angle = (sign * angle) - previous_angle
						deviation = np.sin(delta_angle) * length

						if abs(deviation) > self.max_deviation:
						#elif abs((sign * angle) - previous_angle) > self.max_angle:
							sparse_wp[row_count].append(prev_point)
							# Update the new anchor point
							anchor_point = tuple(prev_point)
							prev_point = tuple(point)

							# Update the "previous angle" to the angle between the
							# new anchor point and the immediately following point
							dx = point[0]-anchor_point[0]
							dy = point[1]-anchor_point[1]
							angle = math.atan2(dy, dx)
							if angle < 0:
								sign = -1
							else:
								sign = 1
							angle = abs(angle)
							while angle > (math.pi/2):
								angle -= (math.pi/2)
							previous_angle = sign * angle
						prev_point = tuple(point)


			sparse_wp[row_count].append(point)
		return sparse_wp

	def test_plot(self,wp,name):
		orig_img = cv2.imread("Evaluation/example.png")
		#x =[]
		#y =[]
		for row in wp:
			for point in row:
				#x.append(point[0])
				#y.append(point[1])
				cv2.circle(orig_img, (point[0],point[1]),1,[150,150,150],2)
		#plt.figure()
		#plt.scatter(x,y)
		#plt.show()
		cv2.imshow(name,orig_img)


if __name__ == '__main__':
	pp = post_processing()
	sparse = pp.drop_redundant_wp(pp.all_waypoints)
	pp.test_plot(pp.all_waypoints, "a")
	pp.test_plot(sparse, "b")
	cv2.waitKey(0)
