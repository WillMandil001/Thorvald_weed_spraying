import rospy
import numpy as np
import cv2
import pickle
import pdb
import math
import copy

class evaluation():
    def __init__(self):

        ''' Our camera sits at 20.5m abover the scene.
        It captures an area of 19.4832m x 34.6368m
        Each pixel is size 0.01804m

        The ground camera captures 0.8448m x 0.4752m
        so we'll use 26.34 pixels

        Wheels are 16cm wide (8.8 pixels)
        and each wheel is 55cm from the robot centre (30.49 pixels)
        '''

        self.camera_FoV = 26 # in pixels
        self.wheel_centre_distance = 30
        self.wheel_width = 9
        img = cv2.imread('Evaluation/world4_annotated.png', 0) # annotated image
        ret,self.annotated_img = cv2.threshold(img,50,1,cv2.THRESH_BINARY)

        with open('Evaluation/toponav.waypoints', 'rb') as wp_file:
            self.all_waypoints = pickle.load(wp_file)

    def evaluate_camera(self):
        vis = cv2.imread('world4.png')
        comp = cv2.imread('Evaluation/world4_annotated.png')
        path_img = copy.deepcopy(self.annotated_img)
        ret,path_img = cv2.threshold(path_img,255,1,cv2.THRESH_BINARY)
        previous_point = []
        for i,row in enumerate(self.all_waypoints):
            if (i % 2) == 0:
                row.reverse()
            for point in row:
                if not previous_point:
                    previous_point = tuple(point)
                else:
                    cv2.line(path_img, previous_point, tuple(point), (255,255,255), self.camera_FoV)
                    cv2.line(vis, previous_point, tuple(point), (255,255,255), self.camera_FoV)
                    cv2.line(comp, previous_point, tuple(point), (255,0,0), self.camera_FoV)
                    previous_point = tuple(point)
        #cv2.imshow('path',path_img)
        cv2.imshow('vis',vis)
        cv2.imshow('c',comp)
        k = cv2.waitKey(0)
        return path_img


    def evaluate_wheels(self):
        vis2 = cv2.imread('world4.png')
        comp2 = cv2.imread('Evaluation/world4_annotated.png')
        wheel_img = copy.deepcopy(self.annotated_img)
        ret,wheel_img = cv2.threshold(wheel_img,50,1,cv2.THRESH_BINARY)

        previous_point = []
        for i,row in enumerate(self.all_waypoints):
            if (i % 2) == 0:
                pass
                #row.reverse()
            for point in row:
                if not previous_point:
                    previous_point = tuple(point)
                else:
                    delta_x = point[0] - previous_point[0]
                    delta_y = point[1] - previous_point[1]

                    L = math.sqrt(delta_x**2 + delta_y **2)
                    dx = int(delta_y * self.wheel_centre_distance/L)
                    dy = int(delta_x * -self.wheel_centre_distance/L)

                    cv2.line(wheel_img, (previous_point[0] - dx, previous_point[1] - dy), (point[0] - dx, point[1] -dy), (255,255,255), self.wheel_width)
                    cv2.line(wheel_img, (previous_point[0] + dx, previous_point[1] + dy), (point[0] + dx, point[1] +dy), (255,255,255), self.wheel_width)
                    cv2.line(vis2, (previous_point[0] - dx, previous_point[1] - dy), (point[0] - dx, point[1] -dy), (255,255,255), self.wheel_width)
                    cv2.line(vis2, (previous_point[0] + dx, previous_point[1] + dy), (point[0] + dx, point[1] +dy), (255,255,255), self.wheel_width)
                    cv2.line(comp2, (previous_point[0] - dx, previous_point[1] - dy), (point[0] - dx, point[1] -dy), (255,0,0), self.wheel_width)
                    cv2.line(comp2, (previous_point[0] + dx, previous_point[1] + dy), (point[0] + dx, point[1] +dy), (255,0,0), self.wheel_width)

                    previous_point = tuple(point)

        #cv2.imshow('wheels',wheel_img)
        cv2.imshow('wheels',vis2)
        cv2.imshow('c2',comp2)
        k = cv2.waitKey(0)
        return wheel_img

    def compute_overlap(self, camera_im, wheel_im):
        print('crop area', sum(sum(self.annotated_img)))
        camera_overlap = cv2.bitwise_and(self.annotated_img, camera_im)
        wheel_overlap = cv2.bitwise_and(self.annotated_img, wheel_im)
        print(sum(sum(camera_overlap)))
        print(sum(sum(wheel_overlap)))

if __name__ == '__main__':
    eval = evaluation()
    camera_path = eval.evaluate_camera()
    wheel_path = eval.evaluate_wheels()
    eval.compute_overlap(camera_path,wheel_path)
