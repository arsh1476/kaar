#!/usr/bin/env python3

import cv2
import numpy as np
import os

# Create output folder
output_dir = "/data/output"
os.makedirs(output_dir, exist_ok=True)

# Load image
img = cv2.imread("lane_frame.png")

if img is None:
    print("Image not found. Make sure lane_frame.png is in this folder.")
    exit()

# Crop road area
height, width, _ = img.shape
cropped = img[int(height * 0.45):height, 0:width]

cv2.imwrite(output_dir + "/01_cropped_road.png", cropped)

# -------------------------------
# 1. Canny Edge Experiments
# -------------------------------

gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (5, 5), 0)

canny_values = [
    (50, 100),
    (100, 200),
    (150, 300)
]

for low, high in canny_values:
    edges = cv2.Canny(blur, low, high)
    filename = output_dir + f"/canny_low_{low}_high_{high}.png"
    cv2.imwrite(filename, edges)
    print("Saved:", filename)

# Use balanced Canny output for Hough experiment
balanced_edges = cv2.Canny(blur, 100, 200)

# -------------------------------
# 2. Hough Transform Experiments
# -------------------------------

hough_values = [10, 30, 60]

for min_len in hough_values:
    hough_img = cropped.copy()

    lines = cv2.HoughLinesP(
        balanced_edges,
        rho=1,
        theta=np.pi / 180,
        threshold=30,
        minLineLength=min_len,
        maxLineGap=20
    )

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(hough_img, (x1, y1), (x2, y2), (0, 255, 0), 3)

    filename = output_dir + f"/hough_minLineLength_{min_len}.png"
    cv2.imwrite(filename, hough_img)
    print("Saved:", filename)

# -------------------------------
# 3. HSV Yellow Detection
# -------------------------------

hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

lower_yellow_hsv = np.array([15, 80, 80])
upper_yellow_hsv = np.array([40, 255, 255])

yellow_mask_hsv = cv2.inRange(hsv, lower_yellow_hsv, upper_yellow_hsv)
yellow_hsv_result = cv2.bitwise_and(cropped, cropped, mask=yellow_mask_hsv)

cv2.imwrite(output_dir + "/hsv_yellow_detection.png", yellow_hsv_result)
print("Saved:", output_dir + "/hsv_yellow_detection.png")

# -------------------------------
# 4. RGB Yellow Detection
# -------------------------------

rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)

lower_yellow_rgb = np.array([120, 100, 0])
upper_yellow_rgb = np.array([255, 255, 120])

yellow_mask_rgb = cv2.inRange(rgb, lower_yellow_rgb, upper_yellow_rgb)
yellow_rgb_result = cv2.bitwise_and(cropped, cropped, mask=yellow_mask_rgb)

cv2.imwrite(output_dir + "/rgb_yellow_detection.png", yellow_rgb_result)
print("Saved:", output_dir + "/rgb_yellow_detection.png")

# -------------------------------
# 5. Lighting Condition Experiment
# -------------------------------

dark_img = cv2.convertScaleAbs(cropped, alpha=0.5, beta=0)
bright_img = cv2.convertScaleAbs(cropped, alpha=1.4, beta=30)

cv2.imwrite(output_dir + "/dark_image.png", dark_img)
cv2.imwrite(output_dir + "/bright_image.png", bright_img)

def detect_lines_for_lighting(image, name):
    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
    edges_img = cv2.Canny(blur_img, 100, 200)

    lines_img = image.copy()

    lines = cv2.HoughLinesP(
        edges_img,
        rho=1,
        theta=np.pi / 180,
        threshold=30,
        minLineLength=30,
        maxLineGap=20
    )

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(lines_img, (x1, y1), (x2, y2), (0, 255, 0), 3)

    cv2.imwrite(output_dir + "/" + name, lines_img)
    print("Saved:", output_dir + "/" + name)

detect_lines_for_lighting(dark_img, "dark_lighting_line_detection.png")
detect_lines_for_lighting(bright_img, "bright_lighting_line_detection.png")

print("\nAll experiment images saved in /data/output")
