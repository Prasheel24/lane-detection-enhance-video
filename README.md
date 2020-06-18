# Video Enhancing, Lane Detection and Turn Prediction

## Authors
Prasheel Renkuntla
Shubham Sonawane
Raj Prakash Shinde
 
## Description
The project demonstrates Video enhancing and Lane detection for self driving cars on the KITTI and Udacity dataset.

## Dependencies
* Ubuntu 16
* Python 3.7
* OpenCV 4.2
* Numpy
* copy
* sys
* argparse

## Run
To run the Video enhancement on Night ride video

```
python3.7 P1-video_correction.py
```
To run the lane detection on images, 2(a) Udacity Dataset -
```
python3.7 P2-1-Homography.py
```
To run the lane detection on challenge video, 2(b) KITTI Dataset -
```
python3.7 P2-2-challenge_accepted.py
```

## Demo
The original night ride frame is shown on the left and the CLAHE processed frame is on the right for the image given below-
<p align="center">
<h5>Comparison of original and enhanced video output</h5>
<img src="/Output/Comparison.png" width="70%">
</p>

For the Udacity Dataset, we have determined the homography points from the image as shown below-
<p align="center">
<h5>Homography points on the image</h5>
<img src="/Output/homo_points.png" width="70%">
</p>

The final output for turn prediction and lane detection is given below-
<p align="center">
<h5>Udacity Dataset</h5>
<img src="/Output/overlay_frame.png" width="70%">
</p>

The output from the challenge video is shown here-
<p align="center">
<h5>KITTI Dataset</h5>
<img src="/Output/Turn.png" width="70%">
</p>

The Output folder consists of video results from the KITTI and Udacity datasets only.

## Reference
* https://stackoverflow.com/questions/19890054/how-to-sharpen-an-image-in-opencv
* https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_filtering/py_filtering.html
* https://www.pyimagesearch.com/2015/10/05/opencv-gamma-correction/
* https://medium.com/@rndayala/image-histograms-in-opencv-40ee5969a3b7
* https://answers.opencv.org/question/193276/how-to-change-brightness-of-an-image-increase-or-decrease/
* https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_histograms/py_histogram_equalization/py_histogram_equalization.html
* https://www.programcreek.com/python/example/89353/cv2.createCLAHE
* http://amroamroamro.github.io/mexopencv/opencv/clahe_demo_gui.html
* https://en.wikipedia.org/wiki/Kernel_(image_processing)
* https://answers.opencv.org/question/175912/how-to-display-multiple-images-in-one-window/
