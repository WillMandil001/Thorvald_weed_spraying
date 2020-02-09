#!/usr/bin/env python

import cv2
import numpy as np
import matplotlib.pyplot as plt
import copy
import os
from sklearn.metrics import confusion_matrix


class CanopyClass():

    def showPlot(self, img):
        for i, col in enumerate(['b', 'g', 'r']):
            hist = cv2.calcHist([img], [i], None, [256], [0, 256])
            plt.plot(hist, color=col)
            plt.xlim([0, 256])
        plt.show()

    def filter_colors(self, cv_image, runtype):
        self.runtype = runtype

        self.circle_color = (255, 0, 0)
        self.tmpfound_rectangle_color = (0, 0, 255)
        self.foundrectangle_color = (255, 0, 0)
        self.contour_color = (0, 255, 0)
        if '_inv' in self.runtype:
            self.circle_color = (255, 255, 255)
            self.tmpfound_rectangle_color = (0, 0, 255)
            self.foundrectangle_color = (255, 0, 0)
            self.contour_color = (0, 0, 255)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        if self.runtype == 'simple':
            # hsv = cv2.blur(hsv, (10, 10))
            hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
            lower_filter = np.array([30, 120, 0])
            upper_filter = np.array([50, 180, 200])
        if self.runtype == 'simple_inv':
            # hsv = cv2.blur(hsv, (10, 10))
            hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
            lower_filter = np.array([0, 0, 0])
            upper_filter = np.array([255, 80, 255])
        elif self.runtype == 'realeasy':
            hsv = cv2.blur(hsv, (40, 40))
            hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
            lower_filter = np.array([0, 10, 40])
            upper_filter = np.array([60, 150, 255])
        elif self.runtype == 'realeasy_inv':
            # hsv = cv2.blur(hsv, (40, 40))
            hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
            lower_filter = np.array([30, 30, 0])
            upper_filter = np.array([100, 90, 40])
        elif self.runtype == 'realhard':
            hsv = cv2.blur(hsv, (40, 40))
            # hsv = cv2.GaussianBlur(hsv, ksize=(17,17), sigmaX=10)
            lower_filter = np.array([40, 40, 0])
            upper_filter = np.array([100, 80, 200])
        elif self.runtype == 'realhard_inv':
            hsv = cv2.blur(hsv, (40, 40))
            hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
            lower_filter = np.array([0, 90, 0])
            upper_filter = np.array([255, 100, 255])
        elif self.runtype == 'ground':
            # hsv = cv2.blur(hsv, (40, 40))
            hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
            lower_filter = np.array([0, 30, 30])
            upper_filter = np.array([20, 140, 80])
        elif self.runtype == 'ground_inv':
            # hsv = cv2.blur(hsv, (40, 40))
            hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
            lower_filter = np.array([30, 0, 10])
            upper_filter = np.array([90, 255, 150])

        mask = cv2.inRange(hsv, lower_filter, upper_filter)
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        # cv2.imshow('hsv', hsv)
        return res, mask

    def get_boxes(self, contours, cv_image):
        rects = []
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            rects.append([x, y, w, h])
            rects.append([x, y, w, h])
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), self.tmpfound_rectangle_color, 2)

        contours_points = []
        contours_boxes, weights = cv2.groupRectangles(rects, 1, 0.2)
        for rect in contours_boxes:
            middle = (x + w / 2, y + h / 2)
            contours_points.append(middle)
            cv2.circle(cv_image, middle, 7, self.circle_color, -1)
            x, y, w, h = rect
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), self.foundrectangle_color, 1)
        return cv_image, contours_boxes, contours_points

    def get_contours(self, res, cv_image):
        # Grayscale
        gray_res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

        # Find Contours
        ret, edged = cv2.threshold(gray_res, 20, 255, 0)
        resultof_find = cv2.findContours(
            edged,
            cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(resultof_find) == 2:
            contours, hierarchy = resultof_find
        elif len(resultof_find) == 3:
            im2, contours, hierarchy = resultof_find

        # Filter contours that are larger than a threshold
        contours_image = np.copy(cv_image)
        threshold_area = 300
        filtered_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > threshold_area:
                filtered_contours.append(cnt)

        cv2.drawContours(contours_image, filtered_contours, -1, self.contour_color, -1)
        boxes_image, contours_boxes, contours_points = self.get_boxes(
            filtered_contours,
            contours_image)

        return boxes_image, filtered_contours, contours_boxes, contours_points

    def get_mask_from_contours(self, contours, old_mask):
        mask = np.copy(old_mask)
        mask = np.where(mask == 255, 0, mask)
        cv2.drawContours(mask, contours, -1, 255, -1)
        return mask

    def compare_masks(self, contours_image, manualmask_location):
        # percent = None
        manualmask_image = cv2.imread(manualmask_location)
        manualmask_image = np.where(manualmask_image != 255, 0, manualmask_image)
        manualmask_image = (255 - manualmask_image)
        manualmask_image = np.mean(manualmask_image, axis=2)
        manualmask_image = np.where(manualmask_image > 0, 255, manualmask_image)

        # import ipdb
        # ipdb.set_trace()

        # print(manualmask_location)
        # plt.subplot(2, 1, 1)
        # plt.imshow(contours_image)
        # plt.subplot(2, 1, 2)
        # plt.imshow(manualmask_image)
        # plt.show()

        result = confusion_matrix(contours_image.ravel(), manualmask_image.ravel()).ravel()
        if len(result) == 1:
            tn, fp, fn, tp = result[0], 0, 0, 0
        elif len(result) == 4:
            tn, fp, fn, tp = result

        return manualmask_image, tn, fp, fn, tp

if __name__ == "__main__":
    debug = False
    showPlot = False
    dataset = '/home/bkbilly/Downloads/Paper_Dataset/'
    plants = ['Anions', 'Lettuce', 'Basil']
    planttypes = ['realhard', 'simple', 'realeasy']
    for num, plant_loc in enumerate(plants):
        plant_location = '{}{}'.format(dataset, plant_loc)

        w_tn_sum, w_fp_sum, w_fn_sum, w_tp_sum = 0., 0., 0., 0.
        w_sizesum = 0.
        p_tn_sum, p_fp_sum, p_fn_sum, p_tp_sum = 0., 0., 0., 0.
        p_sizesum = 0.

        print('-----==== {} ====-----'.format(plant_loc))
        for pngimage in os.listdir('{}/Original'.format(plant_location)):
            inputimage = [
                '{}/Original/{}'.format(plant_location, pngimage),
                '{}_inv'.format(planttypes[num]),
                planttypes[num]
            ]
            # inputimage = ['../images/plants2/train/ros_plant2_1.jpg', 'realeasy_inv', 'realeasy']
            # inputimage = ['../images/plants3/ros_plant3_0.jpg', 'realhard_inv', 'realhard']
            # inputimage = ['../images/plants1/ros_plant0_2.jpg', 'simple_inv', 'simple']
            # inputimage = ['images/plants1/ros_plant0.jpg', 'simple_inv', 'simple']
            # inputimage = ['images/plants1/ros_plant0_1.jpg', 'simple_inv', 'simple']
            # inputimage = ['images/plants1/ros_plant0_2.jpg', 'simple_inv', 'simple']
            # inputimage = ['images/plants2/train/ros_plant2_0.jpg', 'realeasy_inv', 'realeasy']
            # inputimage = ['images/plants2/train/ros_plant2_1.jpg', 'realeasy_inv', 'realeasy']
            cv_image = cv2.imread(inputimage[0])
            # hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            can = CanopyClass()
            # showPlot(cv_image)
            # ground, ground_mask = can.filter_colors(cv_image, 'ground')
            prefilter_image = np.copy(cv_image)

            # Remove Ground
            ground_inv, ground_inv_mask = can.filter_colors(cv_image, 'ground_inv')

            # Get contours of weeds
            weed, weed_mask = can.filter_colors(ground_inv, inputimage[1])
            contours_image, weed_contours, contours_boxes, contours_points = can.get_contours(weed, cv_image)
            indices = np.where(weed_mask == 255)
            weedmask_image = prefilter_image[indices[0], indices[1], :] = (0, 0, 255)
            weed_mask_contours = can.get_mask_from_contours(weed_contours, weed_mask)
            manualmask_location = '{}{}/Masks/Mask_Weed/{}'.format(dataset, plant_loc, pngimage)
            w_manualmask_image, w_tn, w_fp, w_fn, w_tp = can.compare_masks(weed_mask_contours, manualmask_location)

            # Get contours of plants
            plant, plant_mask = can.filter_colors(ground_inv, inputimage[2])
            plants_image, plant_contours, plants_boxes, plants_points = can.get_contours(plant, contours_image)
            indices = np.where(plant_mask == 255)
            prefilter_image[indices[0], indices[1], :] = (0, 255, 0)
            plant_mask_contours = can.get_mask_from_contours(plant_contours, plant_mask)
            manualmask_location = '{}{}/Masks/Mask_Crop/{}'.format(dataset, plant_loc, pngimage)
            p_manualmask_image, p_tn, p_fp, p_fn, p_tp = can.compare_masks(plant_mask_contours, manualmask_location)

            w_tn_sum += w_tn
            w_fp_sum += w_fp
            w_fn_sum += w_fn
            w_tp_sum += w_tp
            w_sizesum += weed_mask_contours.size


            p_tn_sum += p_tn
            p_fp_sum += p_fp
            p_fn_sum += p_fn
            p_tp_sum += p_tp
            p_sizesum += plant_mask_contours.size


            # Save the images (1 time thing)
            # save_location1 = '{}{}/ColourBased/{}'.format(dataset, plant_loc, pngimage)
            # save_location2 = '{}{}/ColourBased/filtered_{}'.format(dataset, plant_loc, pngimage)
            # print(save_location1)
            # print(save_location2)
            # cv2.imwrite(save_location1, prefilter_image)
            # cv2.imwrite(save_location2, plants_image)

            if debug:
                print('{}:'.format(pngimage))
                print('  Weed:  tn={}, fp={} fn={} tp={}'.format(w_tn, w_fp, w_fn, w_tp))
                print('  Plant: tn={}, fp={} fn={} tp={}'.format(p_tn, p_fp, p_fn, p_tp))
            if showPlot:
                plt.subplot(3, 2, 1)
                plt.imshow(cv_image)
                plt.subplot(3, 2, 2)
                plt.imshow(plants_image)
                plt.subplot(3, 2, 3)
                plt.imshow(weed_mask_contours)
                plt.subplot(3, 2, 4)
                plt.imshow(w_manualmask_image)
                plt.subplot(3, 2, 5)
                plt.imshow(plant_mask_contours)
                plt.subplot(3, 2, 6)
                plt.imshow(p_manualmask_image)
                plt.show()

            # cv2.imshow('Mask', ground_mask + plant_mask)
            # cv2.imshow('Mask_plant', mask_plant)
            # cv2.imshow('final', contours)
            # cv2.imshow('Contours', cv_image)
            # cv2.imshow('Contours', plant_mask_contours)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

        print('---------====== Total ======-------------')
        print('{}:'.format(plant_loc))
        print('  Weed:  tn={} fp={} fn={} tp={} size={}'.format(
            round(w_tn_sum / p_sizesum, 3),
            round(w_fp_sum / p_sizesum, 3),
            round(w_fn_sum / p_sizesum, 3),
            round(w_tp_sum / p_sizesum, 3),
            w_sizesum
        ))
        print('  Plant: tn={} fp={} fn={} tp={} size={}'.format(
            round(p_tn_sum / p_sizesum, 3),
            round(p_fp_sum / p_sizesum, 3),
            round(p_fn_sum / p_sizesum, 3),
            round(p_tp_sum / p_sizesum, 3),
            p_sizesum
        ))
