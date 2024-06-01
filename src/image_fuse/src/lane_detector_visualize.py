#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class RoadLaneDetector:
    def __init__(self):
        self.poly_bottom_width = 0.85
        self.poly_top_width = 0.07
        self.poly_height = 0.4
        self.img_center = None
        self.left_detect = False
        self.right_detect = False
        self.left_m = None
        self.right_m = None
        self.left_b = None
        self.right_b = None

    def filter_colors(self, img_frame):
        img_hsv = cv2.cvtColor(img_frame, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(img_frame, (200, 200, 200), (255, 255, 255))
        yellow_mask = cv2.inRange(img_hsv, (10, 100, 100), (40, 255, 255))

        white_image = cv2.bitwise_and(img_frame, img_frame, mask=white_mask)
        yellow_image = cv2.bitwise_and(img_frame, img_frame, mask=yellow_mask)

        return cv2.addWeighted(white_image, 1.0, yellow_image, 1.0, 0.0)

    def limit_region(self, img_edges):
        height, width = img_edges.shape
        mask = np.zeros_like(img_edges)

        points = np.array([[
            ((1 - self.poly_bottom_width) / 2 * width, height),
            ((1 - self.poly_top_width) / 2 * width, height - self.poly_height * height),
            (width - (1 - self.poly_top_width) / 2 * width, height - self.poly_height * height),
            (width - (1 - self.poly_bottom_width) / 2 * width, height)
        ]], dtype=np.int32)

        cv2.fillPoly(mask, points, 255)
        return cv2.bitwise_and(img_edges, mask)

    def hough_lines(self, img_mask):
        return cv2.HoughLinesP(img_mask, 1, np.pi / 180, 20, minLineLength=10, maxLineGap=20)

    def separate_lines(self, img_edges, lines):
        right_lines = []
        left_lines = []
        self.img_center = img_edges.shape[1] / 2

        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1) if x2 != x1 else np.inf
                if abs(slope) > 0.3:
                    if slope > 0 and x1 > self.img_center and x2 > self.img_center:
                        right_lines.append(line)
                        self.right_detect = True
                    elif slope < 0 and x1 < self.img_center and x2 < self.img_center:
                        left_lines.append(line)
                        self.left_detect = True

        return [right_lines, left_lines]

    def regression(self, separated_lines, img_input):
        output = [None] * 4
        right_points = []
        left_points = []

        if self.right_detect:
            for line in separated_lines[0]:
                for x1, y1, x2, y2 in line:
                    right_points.append((x1, y1))
                    right_points.append((x2, y2))

            if right_points:
                right_vx, right_vy, right_x, right_y = cv2.fitLine(np.array(right_points), cv2.DIST_L2, 0, 0.01, 0.01)
                self.right_m = right_vy / right_vx
                self.right_b = (right_x, right_y)

        if self.left_detect:
            for line in separated_lines[1]:
                for x1, y1, x2, y2 in line:
                    left_points.append((x1, y1))
                    left_points.append((x2, y2))

            if left_points:
                left_vx, left_vy, left_x, left_y = cv2.fitLine(np.array(left_points), cv2.DIST_L2, 0, 0.01, 0.01)
                self.left_m = left_vy / left_vx
                self.left_b = (left_x, left_y)

        y1 = img_input.shape[0]
        y2 = int(y1 * 0.6)

        if self.right_detect:
            right_x1 = int(((y1 - self.right_b[1]) / self.right_m) + self.right_b[0])
            right_x2 = int(((y2 - self.right_b[1]) / self.right_m) + self.right_b[0])
            output[0] = (right_x1, y1)
            output[1] = (right_x2, y2)

        if self.left_detect:
            left_x1 = int(((y1 - self.left_b[1]) / self.left_m) + self.left_b[0])
            left_x2 = int(((y2 - self.left_b[1]) / self.left_m) + self.left_b[0])
            output[2] = (left_x1, y1)
            output[3] = (left_x2, y2)

        return output

    def draw_line(self, img_input, lane):
        poly_points = np.array([lane[2], lane[0], lane[1], lane[3]], dtype=np.int32)
        overlay = img_input.copy()
        cv2.fillConvexPoly(overlay, poly_points, (0, 230, 30))
        cv2.addWeighted(overlay, 0.3, img_input, 0.7, 0, img_input)

        cv2.line(img_input, lane[0], lane[1], (0, 255, 255), 5)
        cv2.line(img_input, lane[2], lane[3], (0, 255, 255), 5)

        return img_input


def image_callback(msg, args):
    road_lane_detector, image_pub = args
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        img_filter = road_lane_detector.filter_colors(cv_image)
        img_gray = cv2.cvtColor(img_filter, cv2.COLOR_BGR2GRAY)
        img_edges = cv2.Canny(img_gray, 50, 150)
        img_mask = road_lane_detector.limit_region(img_edges)
        lines = road_lane_detector.hough_lines(img_mask)

        if lines is not None:
            separated_lines = road_lane_detector.separate_lines(img_mask, lines)
            lane = road_lane_detector.regression(separated_lines, cv_image)
            img_result = road_lane_detector.draw_line(cv_image, lane)
        else:
            img_result = cv_image

        image_pub.publish(bridge.cv2_to_imgmsg(img_result, "bgr8"))
        cv2.imshow("result", img_result)

        if cv2.waitKey(1) == 27:
            rospy.signal_shutdown("ESC pressed")

    except CvBridgeError as e:
        rospy.logerr("cv_bridge exception: %s", e)


def main():
    rospy.init_node('road_lane_detector')
    road_lane_detector = RoadLaneDetector()

    bridge = CvBridge()
    first_msg = rospy.wait_for_message('/usb_cam/image_raw', Image)
    cv_image = bridge.imgmsg_to_cv2(first_msg, "bgr8")

    codec = cv2.VideoWriter_fourcc(*'XVID')
    fps = 25.0
    filename = './result.avi'
    writer = cv2.VideoWriter(filename, codec, fps, (cv_image.shape[1], cv_image.shape[0]), True)

    if not writer.isOpened():
        print("Cannot save the video.")
        return -1

    image_pub = rospy.Publisher('/usb_cam/image_raw', Image, queue_size=10)
    image_transport = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback, (road_lane_detector, image_pub))

    cv2.namedWindow("result")
    rospy.spin()


if __name__ == '__main__':
    main()