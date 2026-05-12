#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()

        # Change this if rosbag info shows a different topic name
        IMAGE_TOPIC = "/akandb/camera_node/image/compressed"

        rospy.init_node("my_lane_detector", anonymous=True)

        self.image_sub = rospy.Subscriber(
            IMAGE_TOPIC,
            CompressedImage,
            self.image_callback,
            queue_size=1
        )

        rospy.loginfo("Lane detector node started.")
        rospy.loginfo("Subscribed to: " + IMAGE_TOPIC)

    def draw_hough_lines(self, image, lines, color):
        output = image.copy()

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(output, (x1, y1), (x2, y2), color, 3, cv2.LINE_AA)
                cv2.circle(output, (x1, y1), 3, (0, 255, 0), -1)
                cv2.circle(output, (x2, y2), 3, (0, 0, 255), -1)

        return output

    def image_callback(self, msg):
        try:
            # Convert ROS compressed image to OpenCV BGR image
            img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

            height, width, channels = img.shape

            # 1. Crop image so only road area is visible
            # Duckietown road is usually in lower half of image
            crop_start_y = int(height * 0.45)
            cropped = img[crop_start_y:height, 0:width]

            # 2. Convert cropped image from BGR to HSV
            hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

            # 3. White color filtering
            # White has low saturation and high value
            lower_white = np.array([0, 0, 160])
            upper_white = np.array([180, 70, 255])
            white_mask = cv2.inRange(hsv, lower_white, upper_white)
            white_filtered = cv2.bitwise_and(cropped, cropped, mask=white_mask)

            # 4. Yellow color filtering
            # Yellow hue range is usually around 20-35 in OpenCV HSV
            lower_yellow = np.array([15, 80, 80])
            upper_yellow = np.array([40, 255, 255])
            yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            yellow_filtered = cv2.bitwise_and(cropped, cropped, mask=yellow_mask)

            # 5. Clean masks using morphology
            kernel = np.ones((5, 5), np.uint8)

            white_mask_clean = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
            white_mask_clean = cv2.morphologyEx(white_mask_clean, cv2.MORPH_CLOSE, kernel)

            yellow_mask_clean = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
            yellow_mask_clean = cv2.morphologyEx(yellow_mask_clean, cv2.MORPH_CLOSE, kernel)

            # 6. Canny edge detector on cropped image
            gray_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray_cropped, (5, 5), 0)
            canny_edges = cv2.Canny(blur, 80, 160)

            # 7. Apply Canny to white and yellow masks before Hough
            white_edges = cv2.Canny(white_mask_clean, 50, 150)
            yellow_edges = cv2.Canny(yellow_mask_clean, 50, 150)

            # 8. Hough Transform for white lane lines
            white_lines = cv2.HoughLinesP(
                white_edges,
                rho=1,
                theta=np.pi / 180,
                threshold=25,
                minLineLength=20,
                maxLineGap=20
            )

            # 9. Hough Transform for yellow lane lines
            yellow_lines = cv2.HoughLinesP(
                yellow_edges,
                rho=1,
                theta=np.pi / 180,
                threshold=20,
                minLineLength=15,
                maxLineGap=20
            )

            # 10. Draw Hough lines on cropped image
            final_output = cropped.copy()

            # White lane Hough lines shown in blue
            final_output = self.draw_hough_lines(final_output, white_lines, (255, 0, 0))

            # Yellow lane Hough lines shown in green
            final_output = self.draw_hough_lines(final_output, yellow_lines, (0, 255, 0))

            # 11. Convert filtered images back to RGB for demonstration
            # OpenCV imshow expects BGR, so we convert RGB back to BGR only for display
            white_rgb = cv2.cvtColor(white_filtered, cv2.COLOR_BGR2RGB)
            yellow_rgb = cv2.cvtColor(yellow_filtered, cv2.COLOR_BGR2RGB)

            white_display = cv2.cvtColor(white_rgb, cv2.COLOR_RGB2BGR)
            yellow_display = cv2.cvtColor(yellow_rgb, cv2.COLOR_RGB2BGR)

            # 12. Show output windows
            cv2.imshow("1 Cropped Road Image", cropped)
            cv2.imshow("2 White Filtered Lane Markers", white_display)
            cv2.imshow("3 Yellow Filtered Lane Markers", yellow_display)
            cv2.imshow("4 Canny Edges", canny_edges)
            cv2.imshow("5 White Hough Input Edges", white_edges)
            cv2.imshow("6 Yellow Hough Input Edges", yellow_edges)
            cv2.imshow("7 Final Lane Detection Output", final_output)

            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Error in image_callback: " + str(e))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
