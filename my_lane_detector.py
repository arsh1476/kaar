#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage


class MyLaneDetector:
    def __init__(self):

        rospy.init_node("my_lane_detector_node", anonymous=True)

        self.robot_name = "mybota002410"

        self.image_topic = "/" + self.robot_name + "/camera_node/image/compressed"

        rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1
        )

        rospy.loginfo("My Lane Detector started.")
        rospy.loginfo("Subscribed to: %s", self.image_topic)

        rospy.spin()

    def image_callback(self, msg):
        try:
            # Convert compressed ROS image to OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img is None:
                rospy.logwarn("Image could not be decoded.")
                return

            # Print image details once in a while
            rospy.loginfo_throttle(5, "Image shape: %s", str(img.shape))

            # Resize image if needed
            # Original image is usually 480x640
            height, width, channels = img.shape

            # Crop lower half of image because lane lines are mostly near the ground
            cropped = img[int(height / 2):height, 0:width]

            # Convert image from BGR to HSV
            hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

            # Yellow lane filter
            lower_yellow = np.array([20, 80, 80])
            upper_yellow = np.array([40, 255, 255])
            yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # White lane filter
            lower_white = np.array([0, 0, 180])
            upper_white = np.array([180, 60, 255])
            white_mask = cv2.inRange(hsv, lower_white, upper_white)

            # Combine yellow and white masks
            combined_mask = cv2.bitwise_or(yellow_mask, white_mask)

            # Remove noise using erosion and dilation
            kernel = np.ones((5, 5), np.uint8)
            cleaned_mask = cv2.erode(combined_mask, kernel, iterations=1)
            cleaned_mask = cv2.dilate(cleaned_mask, kernel, iterations=2)

            # Edge detection
            edges = cv2.Canny(cleaned_mask, 50, 150)

            # Hough Transform to detect lines
            lines = cv2.HoughLinesP(
                edges,
                rho=1,
                theta=np.pi / 180,
                threshold=30,
                minLineLength=30,
                maxLineGap=20
            )

            # Draw lines on cropped image
            output = self.output_lines(cropped, lines)

            # Show images
            cv2.imshow("Original Image", img)
            cv2.imshow("Cropped Image", cropped)
            cv2.imshow("Lane Colour Mask", cleaned_mask)
            cv2.imshow("Canny Edges", edges)
            cv2.imshow("Detected Lane Lines", output)

            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Error in image callback: %s", str(e))

    def output_lines(self, original_image, lines):
        output = np.copy(original_image)

        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                x1, y1, x2, y2 = l

                cv2.line(output, (x1, y1), (x2, y2), (255, 0, 0), 2, cv2.LINE_AA)
                cv2.circle(output, (x1, y1), 3, (0, 255, 0), -1)
                cv2.circle(output, (x2, y2), 3, (0, 0, 255), -1)

        return output


if __name__ == "__main__":
    try:
        MyLaneDetector()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
