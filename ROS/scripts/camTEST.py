#!/usr/bin/python3
import cv2
import time
# Initialize important vars
# width  = 240                      # desired width in pixels
# height = 160                      # desired height in pixels
width = 176                         # desired width in pixels
height = 144                         # desired height in pixels
camera = cv2.VideoCapture(0)        # Take images from the camera assigned as "0"


# A function to capture an image & return all pixel data
def newImage(size=(width, height)):
    ret, image = camera.read()          # return the image as well as ret
    if not ret:                         # (ret is a boolean for returned successfully?)
        print("NO IMAGE")
        return None                     # (return an empty var if the image could not be captured)
    image = cv2.resize(image, size)     # reduce size of image
    return image


if __name__ == "__main__":
    num_frames = 0
    while True:
        try:
           if num_frames == 0:
              start = time.time()
           image = newImage()
           print(image.shape)          # print info about the image (height, width, colorspace)
           
           #cv2.imshow('webcam', image)
           num_frames = num_frames + 1
           cv2.waitKey(1)

        except KeyboardInterrupt:
           break


    end = time.time()
    sec = end - start
    print("  FPS: " + str(int(num_frames/sec)))
    cv2.destroyAllWindows()
