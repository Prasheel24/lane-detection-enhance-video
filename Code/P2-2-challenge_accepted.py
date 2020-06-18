import numpy as np
import sys
# This try-catch is a workaround for Python3 when used with ROS; 
# it is not needed for most platforms
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass

import cv2 as cv
import numpy as np
from random import randint
import matplotlib.pyplot as plt
import time

K=np.array([[903.7596, 0 , 695.7519]
, [0, 901.9653, 224.2509],
 [0, 0, 1]])
D = np.array([[-3.639558e-01, 1.788651e-01, 6.029694e-04, -3.922424e-04, -5.382460e-02]])


def darken_grayscale(frame, gamma = 1.0):
  inv = 1.0/gamma
  table = np.array([((i / 255.0) ** inv) * 255
      for i in np.arange(0, 256)]).astype("uint8")
  return cv.LUT(frame, table)

def crop_image(img):
    height, width = img.shape
    cropped = img[int(height/2):height, 0:width]
    return cropped 

def crop_image_mask1(img):
    height, width = img.shape
    cropped = img[0:220, 0:156]
    return cropped 

def warp(h_matrix, contour, source, copy):
    coordinates = np.indices((copy.shape[1], copy.shape[0]))
    coordinates = coordinates.reshape(2, -1)
    coordinates = np.vstack((coordinates, np.ones(coordinates.shape[1])))
    temp_x, temp_y = coordinates[0], coordinates[1]
    warp_coordinates = h_matrix@coordinates
    x1, y1 ,z= warp_coordinates[0, :]/warp_coordinates[2, :], warp_coordinates[1, :]/warp_coordinates[2, :], warp_coordinates[2, :]/warp_coordinates[2, :]
    temp_x, temp_y = temp_x.astype(int),temp_y.astype(int)
    x1, y1 = x1.astype(int), y1.astype(int)

    if x1.all() >= 0 and x1.all() < 1392 and y1.all() >= 0 and y1.all() < 512:
        source[y1, x1] = copy[temp_y, temp_x]
    return source

vid = cv.VideoCapture("challenge_video.mp4")
frame_width = int(vid.get(3))
frame_height = int(vid.get(4))
# out = cv.VideoWriter('challenge_accepted_video.avi', cv.VideoWriter_fourcc('M','J','P','G'), 30, (frame_width, frame_height))

while(vid.isOpened()):
    _, frame = vid.read()
    if frame is not None:
      frame_copy = np.copy(frame)
      frame=cv.undistort(frame,K,D,None,K)

      font = cv.FONT_HERSHEY_SIMPLEX
      kernel = np.ones((5,5),np.uint8)
      kernel1 = np.ones((5,5),np.uint8)
      hsv = cv.cvtColor(frame, cv.COLOR_BGR2HLS)

      # Create Mask
      mask1 = cv.inRange(hsv, np.array([10, 100, 20]),np.array([40, 255, 200]))
      mask2 = cv.inRange(hsv, np.array([0, 200, 0]),np.array([255, 255, 200]))
      mask1 = cv.dilate(mask1,kernel,iterations = 1)
      mask1 = cv.morphologyEx(mask1, cv.MORPH_OPEN, kernel)
      mask1 = cv.erode(mask1,kernel,iterations = 1)
      mask2 = cv.dilate(mask2,kernel1,iterations = 1)
      mask2 = cv.morphologyEx(mask2, cv.MORPH_CLOSE, kernel1)
      mask = cv.bitwise_or(mask1,mask2)

      ref_frame = np.array([[0,0], [400,0], [400,400], [0,400]])
      img_frame = np.array([[598,465], [721,465], [1263,700], [22,700]])

      # Homography                     
      H,_=cv.findHomography(img_frame, ref_frame, cv.RANSAC, 3.0)    
      res = cv.warpPerspective(mask, H, (600,400))

      H,_=cv.findHomography(img_frame, ref_frame, cv.RANSAC, 3.0)    
      res1 = cv.warpPerspective(frame, H, (400,400))
      

      # Find Histogram
      histogram=np.sum(res[res.shape[0]//2:,:], axis=0)
      midpoint=np.int(histogram.shape[0]/2)
      left_start=np.argmax(histogram[:midpoint])
      right_start=np.argmax(histogram[midpoint:])+midpoint
      location=[left_start,right_start,midpoint]
      
      if left_start==0:
        left_start=start1
      if right_start==0:
        right_start=start2
      xl=[]
      yl=[]
      xr=[]
      yr=[]
      poly_left=[]
      poly_right=[]
      offset=0
      start1=left_start
      start2=right_start
      i=0
      for j in range(0,100):
        s1=start1
        s2=start2
        xl.append(start1)
        yl.append(400-(2+offset)*j)
        xr.append(start2)
        yr.append(400-(2+offset)*j)
        
        histogram1=np.sum(res[400-(2+offset)*j-2:400-(2+offset)*j,start1-30:start1+30], axis=0)
        histogram2=np.sum(res[400-(2+offset)*j-2:400-(2+offset)*j,start2-30:start2+30], axis=0)

        if len(histogram1)!=0:
          actual1=np.argmax(histogram1)+start1-30
          start1=actual1
          if np.argmax(histogram1)==0:
            start1=s1
        if len(histogram2)!=0:
          actual2=np.argmax(histogram2)+start2-30
          start2=actual2
          if np.argmax(histogram2)==0:
            start2=s2+i

          pts_left = np.array([np.transpose(np.vstack([xl, yl]))])
          pts_right = np.array([np.transpose(np.vstack([xr, yr]))])
          pts = np.hstack((pts_left, pts_right))
          pts_new = np.array([[xl[len(xl)-1], yl[len(yl)-1]], [xr[len(xr)-1], yr[len(yr)-1]], [xl[len(xl)-1], 100], [xr[len(xr)-1], 100] ], np.int32)
          
          cv.line(res1, (xl[len(xl)-1], yl[len(yl)-1]), (xr[len(xr)-1], yr[len(yr)-1]), [255,0,0], 10)
          
          cv.polylines(res1, np.int32([pts_left]), 20, (255,0,0))
          cv.polylines(res1, np.int32([pts_right]), 20, (255,0,0))

      for f in range(len(xl)):
        cv.circle(res1, (xl[f],yl[f]), 2, [0,0,255], 3)
        cv.circle(res1, (xr[f],yr[f]), 2, [0,0,255], 3)
      pts_mid = np.array([(xl[len(xl)-1], yl[len(yl)-1]), (xr[len(xr)-1], yr[len(yr)-1]), (xl[len(xl)-2], yl[len(yl)-2]), (xr[len(xr)-2], yr[len(yr)-2])], np.int32)
      mid_point_line = np.mean(pts_mid)/2

      center_point = 107
      if mid_point_line > center_point:
        # print("Towards Right")
         cv.putText(frame, 'Right', (600,50), font,1, (0,0,255), 3, cv.LINE_AA)
      elif mid_point_line < center_point - 30:
        # print("Towards Left")
         cv.putText(frame, 'Left', (600,50), font,1, (0,0,255), 3, cv.LINE_AA)
      else:
        # print("Going Straight")
         cv.putText(frame, 'Straight', (600,50), font,1, (0,0,255), 3, cv.LINE_AA)
          
      frame=warp(np.linalg.inv(H),img_frame,frame,res1)
      cv.imshow("Frame", frame)
      # out.write(frame)

      k = cv.waitKey(20) & 0xFF
      if k == 27:
         break
    else: 
        break   
# out.release()
vid.release()
cv.destroyAllWindows()