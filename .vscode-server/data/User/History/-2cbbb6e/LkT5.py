import logging
import cv2
import numpy as np
from PIL import Image
import sys

sys.path.append("..")
from MangDang.LCD.ST7789 import ST7789
from api.gif import AnimatedGif

disp = ST7789()
disp.begin()

def take_photo():
    """
    Captures a photo from the webcam and returns it as a PIL Image object.

    Returns:
    - image (PIL.Image): The captured image or None if the webcam is not accessible.
    """
    cap = cv2.VideoCapture(-1)

    if not cap.isOpened():
        return None

    ret, frame = cap.read()
    image = None
    if ret:
        image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    cap.release()
    return image