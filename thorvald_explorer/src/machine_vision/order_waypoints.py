import numpy as np
import rospy
import pdb
from matplotlib import pyplot as plt
import time
import cv2


class order_wp():
	def __init__(self):
		pass

	def order(self, rows, angle):
		# INPUT: A list of lists, where each single list contains waypoints of a seperate row
		# OUPUT: returns the same waypoints, but in the correct order for navigating along the rows 
		order = []
		row_count = 0
		R = np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle), np.cos(angle)]])
		for row in rows:
			order.append([])
			points = []
			column_coord = []
			for point in row:
				wp_vector = np.reshape(np.array(point),[2,1])
				rotated_vector = np.dot(R,wp_vector)
				points.append(rotated_vector)
				column_coord.append(rotated_vector[1])
			order[row_count] = np.argsort(column_coord)
			row_count += 1
		return order

	def test_plot(self, rows, order):

		vis_im = np.zeros([1080, 1920, 3])
		i = 0
		for row in order:
			for index in row:
				ind = index[0]
				cv2.circle(vis_im, (rows[i][ind][0], rows[i][ind][1]), 5, (100,50,50), 3)
			i += 1

		cv2.imshow("order", vis_im)
		k = cv2.waitKey(0)
		if k == 27:
			pass
		cv2.destroyAllWindows()

if __name__ == '__main__':
	rows = [[[721, 278], [731, 271], [751, 270], [741, 271], [771, 270], [761, 270], [791, 270], [781, 271], [811, 270], [801, 270], [831, 269], [821, 270], [851, 268], [841, 268], [871, 269], [861, 268], [891, 270], [881, 269], [911, 272], [901, 271], [931, 272], [921, 272], [951, 272], [941, 272], [971, 272], [961, 272], [991, 268], [1001, 271], [1011, 276], [1021, 282], [1031, 291], [1041, 298], [1051, 307], [1061, 314], [1071, 320], [1081, 328], [1091, 335], [1111, 340], [1101, 338], [1131, 337], [1121, 339], [1141, 332], [1151, 327], [1161, 322], [1171, 316], [1181, 309], [1191, 300], [1201, 293], [1211, 286], [1221, 279], [1231, 274], [1241, 270]], [[721, 416], [731, 408], [751, 408], [741, 409], [771, 408], [761, 408], [791, 408], [781, 408], [811, 408], [801, 408], [831, 407], [821, 408], [851, 406], [841, 406], [871, 407], [861, 406], [891, 408], [881, 407], [911, 409], [901, 409], [931, 410], [921, 410], [951, 410], [941, 409], [971, 410], [961, 409], [991, 409], [1011, 408], [1001, 409], [1031, 408], [1021, 408], [1051, 408], [1041, 408], [1071, 408], [1061, 408], [1091, 407], [1081, 408], [1111, 406], [1101, 406], [1131, 406], [1121, 406], [1151, 407], [1141, 406], [1171, 409], [1161, 409], [1191, 410], [1181, 409], [1211, 410], [1201, 410], [1231, 410], [1221, 410], [1241, 410]], [[721, 549], [731, 541], [751, 541], [741, 541], [771, 541], [761, 541], [791, 541], [781, 541], [811, 541], [801, 541], [831, 540], [821, 540], [851, 538], [841, 539], [871, 539], [861, 538], [891, 541], [881, 539], [911, 542], [901, 542], [931, 542], [921, 543], [951, 543], [941, 542], [971, 542], [961, 543], [991, 541], [1011, 541], [1001, 541], [1031, 541], [1021, 541], [1051, 541], [1041, 541], [1071, 541], [1061, 541], [1091, 540], [1081, 540], [1111, 539], [1101, 539], [1131, 538], [1121, 538], [1151, 540], [1141, 538], [1171, 542], [1161, 542], [1191, 542], [1181, 542], [1211, 542], [1201, 542], [1231, 542], [1221, 543], [1241, 542]], [[721, 694], [731, 686], [751, 685], [741, 686], [761, 681], [771, 675], [781, 670], [791, 665], [811, 660], [801, 661], [831, 665], [821, 662], [841, 668], [851, 672], [861, 676], [881, 681], [871, 680], [901, 685], [891, 683], [921, 687], [911, 686], [941, 686], [931, 686], [961, 687], [951, 687], [971, 687], [991, 687], [1011, 686], [1001, 687], [1031, 686], [1021, 686], [1051, 686], [1041, 686], [1071, 686], [1061, 686], [1091, 685], [1081, 686], [1111, 685], [1101, 684], [1131, 684], [1121, 684], [1151, 686], [1141, 684], [1171, 687], [1161, 687], [1191, 688], [1181, 687], [1211, 688], [1201, 688], [1231, 688], [1221, 688], [1241, 688]], [[721, 819], [731, 811], [751, 811], [741, 812], [771, 811], [761, 811], [791, 811], [781, 811], [811, 811], [801, 811], [831, 810], [821, 811], [851, 808], [841, 809], [871, 809], [861, 809], [891, 811], [881, 810], [911, 812], [901, 812], [931, 812], [921, 813], [951, 813], [941, 812], [971, 813], [961, 812], [991, 809], [1011, 809], [1001, 809], [1031, 809], [1021, 809], [1051, 809], [1041, 809], [1071, 811], [1061, 810], [1091, 816], [1081, 813], [1111, 821], [1101, 818], [1121, 825], [1131, 828], [1151, 834], [1141, 832], [1171, 834], [1161, 835], [1181, 830], [1191, 823], [1201, 818], [1221, 812], [1211, 812], [1241, 809], [1231, 809]], [[731, 470], [751, 470], [741, 470], [771, 470], [761, 470], [791, 470], [781, 470], [811, 471], [801, 470], [831, 473], [821, 472], [851, 474], [841, 474], [871, 472], [861, 473], [891, 471], [881, 472], [911, 471], [901, 471], [931, 471], [921, 471], [951, 472], [941, 472], [971, 471], [961, 470], [991, 470], [981, 463], [1011, 470], [1001, 470], [1031, 470], [1021, 470], [1051, 470], [1041, 470], [1071, 471], [1061, 470], [1091, 473], [1081, 472], [1111, 474], [1101, 474], [1131, 473], [1121, 473], [1151, 472], [1141, 472], [1171, 471], [1161, 471], [1191, 471], [1181, 471], [1211, 471], [1201, 471], [1231, 470], [1221, 474], [1241, 471]], [[731, 605], [751, 605], [741, 605], [771, 605], [761, 605], [791, 605], [781, 605], [811, 606], [801, 606], [831, 608], [821, 607], [851, 609], [841, 609], [871, 607], [861, 608], [891, 606], [881, 607], [911, 606], [901, 606], [931, 606], [921, 606], [951, 607], [941, 607], [971, 606], [961, 606], [991, 605], [981, 598], [1011, 605], [1001, 605], [1031, 605], [1021, 605], [1051, 605], [1041, 605], [1071, 606], [1061, 605], [1091, 609], [1081, 607], [1111, 609], [1101, 609], [1131, 608], [1121, 608], [1151, 607], [1141, 607], [1171, 606], [1161, 606], [1191, 606], [1181, 606], [1211, 607], [1201, 606], [1231, 606], [1221, 607], [1241, 606]], [[731, 740], [751, 740], [741, 740], [771, 740], [761, 740], [791, 741], [781, 740], [811, 742], [801, 740], [831, 743], [821, 742], [851, 744], [841, 744], [871, 742], [861, 743], [891, 741], [881, 742], [911, 741], [901, 742], [931, 741], [921, 741], [951, 742], [941, 742], [971, 741], [961, 741], [991, 744], [981, 733], [1011, 744], [1001, 744], [1031, 743], [1021, 744], [1051, 743], [1041, 743], [1071, 743], [1061, 743], [1091, 743], [1081, 743], [1111, 742], [1101, 742], [1131, 741], [1121, 741], [1151, 743], [1141, 741], [1171, 744], [1161, 744], [1191, 745], [1181, 745], [1211, 745], [1201, 745], [1231, 745], [1221, 745], [1241, 745]]]
	od = order_wp()
	result = od.order(rows, -90)
	od.test_plot(rows, result)