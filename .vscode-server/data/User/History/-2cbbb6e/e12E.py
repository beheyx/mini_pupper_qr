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
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open the webcam.")
        return None

    ret, frame = cap.read()
    image = None
    if ret:
        image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    cap.release()
    return image


def main():
    image = take_photo()

    if image is None:
        print("No image captured")
    else:
        # Test the camera by showing the captured image using OpenCV
        open_cv_image = np.array(image)  # Convert PIL image to OpenCV format
        open_cv_image = cv2.cvtColor(open_cv_image, cv2.COLOR_RGB2BGR)  # Convert RGB to BGR

        # Display the image in a window
        cv2.imshow('Captured Image', open_cv_image)

        # Wait for a key press to close the window
        print("Press any key in the image window to exit.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
