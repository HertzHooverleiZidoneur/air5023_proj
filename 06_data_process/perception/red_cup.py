"""TEAM-06: color detection for visual sanity checks, not semantic grounding."""
import cv2
import numpy as np


def detect_red(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, np.array([0, 110, 55]), np.array([12, 255, 255]))
    mask |= cv2.inRange(hsv, np.array([168, 110, 55]), np.array([179, 255, 255]))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 8:
        return None
    x, y, w, h = cv2.boundingRect(contour)
    return {"bbox_xywh": [x, y, w, h], "area_pixels": float(cv2.contourArea(contour))}
