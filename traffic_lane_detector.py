# ------------------------------------------------------------------------------
# Traffic lane detector.
# Detect traffic lane with OpenCV library.
# Reference: https://github.com/galenballew/SDC-Lane-and-Vehicle-Detection-Tracking/tree/master/Part%20I%20-%20Simple%20Lane%20Detection
# Copyright (c) UIUC
# Licensed under the MIT License.
# Written by Bowen Cheng (bcheng9@illinois.edu)
# ------------------------------------------------------------------------------

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import roslib
import rospy



def canny(img, low_threshold, high_threshold):
    """Applies the Canny transform"""
    return cv2.Canny(img, low_threshold, high_threshold)


def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)


def draw_lines(img, lines, color=[255, 0, 0], thickness=2):
    """
    NOTE: this is the function you might want to use as a starting point once you want to
    average/extrapolate the line segments you detect to map out the full
    extent of the lane (going from the result shown in raw-lines-example.mp4
    to that shown in P1_example.mp4).
    Think about things like separating line segments by their
    slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
    line vs. the right line.  Then, you can average the position of each of
    the lines and extrapolate to the top and bottom of the lane.
    This function draws `lines` with `color` and `thickness`.
    Lines are drawn on the image inplace (mutates the image).
    If you want to make the lines semi-transparent, think about combining
    this function with the weighted_img() function below
    """
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)


def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    `img` should be the output of a Canny transform.
    Returns an image with hough lines drawn.
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, lines)
    return line_img


# Python 3 has support for cool math symbols.
def weighted_img(img, initial_img, α=0.8, β=1., λ=0.):
    """
    `img` is the output of the hough_lines(), An image with lines drawn on it.
    Should be a blank image (all black) with lines drawn on it.
    `initial_img` should be the image before any processing.
    The result image is computed as follows:
    initial_img * α + img * β + λ
    NOTE: initial_img and img must be the same shape!
    """
    return cv2.addWeighted(initial_img, α, img, β, λ)


def main(img):
    # img = cv2.imread('record/frame0000.jpg')
    # img = cv2.cvtColor(cv2.imread('high_bay_lane.jpg'), cv2.COLOR_BGR2RGB)
    # plt.imshow(img)
    # plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
    # plt.show()

    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # plt.imshow(gray_img)
    # plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
    # plt.show()

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100], dtype= 'uint8')
    upper_yellow = np.array([30, 255, 255], dtype='uint8')
    mask_yellow = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
    mask_white = cv2.inRange(gray_img, 200, 255)
    mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
    mask_yw_image = cv2.bitwise_and(gray_img, mask_yw)
    # plt.imshow(mask_yellow)
    # plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
    # plt.show()

    # Use yellow to filter lane.
    lane_mask = mask_yellow
    kernel_size = 5
    gauss_gray = gaussian_blur(lane_mask, kernel_size)
    low_threshold = 50
    high_threshold = 150
    canny_edges = canny(gauss_gray, low_threshold, high_threshold)

    # rho and theta are the distance and angular resolution of the grid in Hough space
    # same values as quiz
    rho = 2
    theta = np.pi / 180
    # threshold is minimum number of intersections in a grid for candidate line to go to output
    threshold = 20
    min_line_len = 50
    max_line_gap = 200

    line_image = hough_lines(canny_edges, rho, theta, threshold, min_line_len, max_line_gap)
    result = weighted_img(line_image, cv2.cvtColor(img, cv2.COLOR_BGR2RGB), α=0.8, β=1., λ=0.)

    # plt.imshow(line_image)
    # plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
    # plt.show()

    # print(line_image.shape, np.unique(line_image[..., 0]), np.unique(line_image[..., 1]), np.unique(line_image[..., 2]))

    H, W, _ = line_image.shape
    H_grid = H // 8
    W_grid = W // 2
    left_image = line_image[6*H_grid:, :W_grid, :]
    right_image = line_image[6*H_grid:, W_grid:, :]

    # plt.imshow(left_image)
    # plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
    # plt.show()
    #
    # plt.imshow(right_image)
    # plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
    # plt.show()

    lane_on_left = np.sum((left_image[..., 0] > 0).astype(np.int))
    lane_on_right = np.sum((right_image[..., 0] > 0).astype(np.int))

    if lane_on_left > lane_on_right:
        # Turn left.
        return -1
    elif lane_on_left < lane_on_right:
        # Turn right.
        return 1
    else:
        return 0

if __name__ == '__main__':
    #img = cv2.imread('record/frame0000.jpg')
    img = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")


    turn = main(img)

    os.system('roslaunch basic_launch dbw_joystick.launch')
    os.system('rostopic true /pacmod/as_rx/enable')

    if turn == -1:
        #os.system('ls -l')
        os.system('rostopic -0.5 as_rx / turn_cmd')

    elif turn == 1:
        print('Turn right.')
        os.system('rostopic 0.5 as_rx / turn_cmd')
    else:
        os.system('rostopic 0 as_rx / turn_cmd')
        print('No turn.')
