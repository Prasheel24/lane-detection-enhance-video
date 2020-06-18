import numpy as np
import sys
# This try-catch is a workaround for Python3 when used with ROS; 
# it is not needed for most platforms
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass

import cv2
import matplotlib.pyplot as plt

vid = cv2.VideoCapture('Night Drive - 2689.mp4')
frame_width = int(vid.get(3))
frame_height = int(vid.get(4))
out = cv2.VideoWriter('video_enhanced.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 30, (frame_width, frame_height))
def main():
    adjusted = None
    while(vid.isOpened()):
        ret, frame = vid.read()
        if frame is not None:

            ref = np.copy(frame)
            # frame = cv2.resize(frame, (0, 0), None, .50, .50)
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

            # Splitting the LAB image to different channels
            l, a, b = cv2.split(lab)
            
            # Applying CLAHE to L-channel---
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            cl = clahe.apply(l)

            # Merge the CLAHE enhanced L-channel with the a and b channel
            limg = cv2.merge((cl, a, b))
            # Converting image from LAB Color model to RGB model
            final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
            median = cv2.medianBlur(final, 5)

            sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
            sharpen = cv2.filter2D(median, -1, sharpen_kernel)
            image_new = cv2.add(sharpen,np.array([50.0]))
            # np_horizontal_concat = np.concatenate((frame, image_new), axis = 1)
            out.write(image_new)
            cv2.imshow("Final", image_new)
            # cv2.imshow("Comparison", np_horizontal_concat)

        else:
            break
        key = cv2.waitKey(1)
        if key == 27:
            break

    out.release()
    vid.release()
    cv2.destroyAllWindows()

main()

# -------Gamma correction pipeline


# def adjust_gamma(image, gamma=1.0):
#     # build a lookup table mapping the pixel values [0, 255] to
#     # their adjusted gamma values
#     invGamma = 1.0 / gamma
#     table = np.array([((i / 255.0) ** invGamma) * 255
#         for i in np.arange(0, 256)]).astype("uint8")
#     # apply gamma correction using the lookup table
#     return cv2.LUT(image, table)


# def main():
#     # print(frame_height, frame_width)
#     adjusted = None
#     while(vid.isOpened()):
#         ret, frame = vid.read()
#         if frame is not None:
#             ref = np.copy(frame)
#             # print("1")
#             # for gamma in np.arange(0.0, 3.5, 0.5):
#             #     # ignore when gamma is 1 (there will be no change to the image)
#             #     if gamma == 1:
#             #         continue
#                 # apply gamma correction and show the images
#             # gamma = gamma if gamma > 0 else 0.1
#             adjusted = adjust_gamma(frame, gamma=3.5)
#                 # cv2.putText(adjusted, "g={}".format(gamma), (10, 30),
#                 #     cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)
#             median = cv2.medianBlur(adjusted,5)
#             # aw = cv2.addWeighted(median, 4, cv2.blur(median, (30, 30)), -4, 128)
#             # edges = cv2.Canny(aw, 100, 300)

#             # lines = cv2.HoughLinesP(edges, 2, np.pi/180.0, 50, minLineLength=10, maxLineGap=100)

#             # # Draw lines on the detected points
#             # for line in lines:
#             #     x1, y1, x2, y2 = line[0]
#             #     cv2.line(ref, (x1, y1), (x2, y2), (0,0,255), 1)


#             # ---Sharpening filter----
#             kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
#             im = cv2.filter2D(median, -1, kernel)
#             gblur = cv2.GaussianBlur(im, (7,7), 0)
#             cv2.imshow("sharpened", gblur)
#             #---Approach 2---
            
#             # cv2.imshow("Add_weighted", aw)


#             # cv2.imshow("Result Image", ref) 
#             # cv2.imshow("Edges",edges)

#             # cv2.imshow("Images", im) #np.hstack([frame, adjusted]))
#             # cv2.waitKey(0)

#         key = cv2.waitKey(1)
#         if key == 27:
#             break

#     vid.release()
#     cv2.destroyAllWindows()

# main()

# https://stackoverflow.com/questions/19890054/how-to-sharpen-an-image-in-opencv
# https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_filtering/py_filtering.html
# https://www.pyimagesearch.com/2015/10/05/opencv-gamma-correction/
# https://medium.com/@rndayala/image-histograms-in-opencv-40ee5969a3b7
# https://answers.opencv.org/question/193276/how-to-change-brightness-of-an-image-increase-or-decrease/
# https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_histograms/py_histogram_equalization/py_histogram_equalization.html
# https://www.programcreek.com/python/example/89353/cv2.createCLAHE
# http://amroamroamro.github.io/mexopencv/opencv/clahe_demo_gui.html
# https://en.wikipedia.org/wiki/Kernel_(image_processing)
# https://answers.opencv.org/question/175912/how-to-display-multiple-images-in-one-window/