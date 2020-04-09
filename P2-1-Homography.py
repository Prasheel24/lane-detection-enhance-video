import numpy as np
import cv2 as cv
import copy
import sys

try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass

K=np.array([[903.7596, 0 , 695.7519]
, [0, 901.9653, 224.2509],
 [0, 0, 1]])

D = np.array([[-3.639558e-01, 1.788651e-01, 6.029694e-04, -3.922424e-04, -5.382460e-02]])

#convert imgs to video frame
#img_array = []
#for filename in glob.glob('C:/Users/shubh/Desktop/PMRO/SEM2/Perception/P2/data_1/data/*.png'):
#    img = cv.imread(filename)
#    height, width, layers = img.shape
#    size = (width,height)
#    img_array.append(img)
#    
#out = cv.VideoWriter('project.avi',cv.VideoWriter_fourcc(*'DIVX'), 15, size)
# 
#for i in range(len(img_array)):
#    out.write(img_array[i])
#out.release()

def find_homography(src, tgt):
	A = np.array([
					[-src[0][0], -src[0][1], -1, 0, 0, 0, src[0][0] * tgt[0][0], src[0][1] * tgt[0][0], tgt[0][0]],
					[0, 0, 0, -src[0][0], -src[0][1], -1, src[0][0] * tgt[0][1], src[0][1] * tgt[0][1], tgt[0][1]],
					[-src[1][0], -src[1][1], -1, 0, 0, 0, src[1][0] * tgt[1][0], src[1][1] * tgt[1][0], tgt[1][0]],
					[0, 0, 0, -src[1][0], -src[1][1], -1, src[1][0] * tgt[1][1], src[1][1] * tgt[1][1], tgt[1][1]],
					[-src[2][0], -src[2][1], -1, 0, 0, 0, src[2][0] * tgt[2][0], src[2][1] * tgt[2][0], tgt[2][0]],
					[0, 0, 0, -src[2][0], -src[2][1], -1, src[2][0] * tgt[2][1], src[2][1] * tgt[2][1], tgt[2][1]],
					[-src[3][0], -src[3][1], -1, 0, 0, 0, src[3][0] * tgt[3][0], src[3][1] * tgt[3][0], tgt[3][0]],
					[0, 0, 0, -src[3][0], -src[3][1], -1, src[3][0] * tgt[3][1], src[3][1] * tgt[3][1], tgt[3][1]],
				])
	U,S,V = np.linalg.svd(A, full_matrices=True)
	V = (copy.deepcopy(V))/(copy.deepcopy(V[8][8]))
	H = V[8,:].reshape(3, 3)
	return H

def darken_grayscale(frame, gamma = 1.0):
    inv = 1.0/gamma
    table = np.array([((i / 255.0) ** inv) * 255
        for i in np.arange(0, 256)]).astype("uint8")
    return cv.LUT(frame, table)

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

def avg_line(lines):
    left = np.empty([1,3])
    right = np.empty([1,3])
 
    if lines is not None:
        for line in lines:
            for x1,y1,x2,y2 in line:
                if x1 != x2:
                    slope = (y2-y1)/(x2-x1)
                    intercept = y1 - slope*x1
                    line_length = np.sqrt((y2-y1)**2+(x2-x1)**2)
                    #reject lines withib +-5 degrees slope
                    if slope < -0.087 :
                        #append negative slope lines to left lane
                        left = np.append(left,np.array([[slope, intercept, line_length]]),axis = 0)
                    elif slope > 0.087:
                        #append positive slope lines to right lane
                        right = np.append(right,np.array([[slope, intercept, line_length]]),axis = 0)

    #reject outliers
    left = left[outlier_remove(left[:,0])]
    right = right[outlier_remove(right[:,0])]

    #calculating average
    left_lane = np.dot(left[1:,2],left[1:,:2])/np.sum(left[1:,2]) if len(left[1:,2]) > 0 else None
    right_lane = np.dot(right[1:,2],right[1:,:2])/np.sum(right[1:,2]) if len(right[1:,2]) > 0 else None

    return left_lane, right_lane

#remove outliers beyond 2 standard deviations
def outlier_remove(obs):
    return np.array(abs(obs - np.mean(obs)) < 2*np.std(obs))

vid = cv.VideoCapture("project.avi")


while(vid.isOpened()):
    alpha = 0.3
    ret, image = vid.read()
    if image is not None:
    
        height, width, _ = image.shape
        #undistort the image using K and D parameters
        frame = cv.undistort(image, K, D, None, K)
   
        font = cv.FONT_HERSHEY_SIMPLEX
        #set size for reference frame and 4 points for homography
        ref_frame = np.array([[0,0], [550,0], [550,550], [0,550]])
        img_frame = np.array([[510,300],[750,300], [900,480],[230,480]])#2 lane dividers
        
        #calculate homography
        h_matrix = find_homography(img_frame, ref_frame)    
        h_inv = find_homography(ref_frame, img_frame)
        new_img = cv.warpPerspective(image, h_matrix, (600,550))
        new_height, new_width, _ = new_img.shape
       
        #convert warped image into grayscale
        gray = cv.cvtColor(new_img, cv.COLOR_RGB2GRAY)  
        #darken the grayed image using gamma correction
        darkened = darken_grayscale(gray, 0.1)
        #apply canny edge detection
        canny = cv.Canny(darkened, 100, 300) 
                
        #draw lines using hough transform
        lines = cv.HoughLinesP(canny, rho=2, theta = np.pi/180, threshold=100, 
                               lines = np.array([]), minLineLength = 20, maxLineGap=300)
        
        #get the average of left and right lines
        left, right = avg_line(lines)
        
        #drawing lines on image
        if left is not None and right is not None:
            slope_l, intercept_l = left
            slope_r, intercept_r = right
            if slope_l is not None and intercept_l is not None and slope_r is not None and intercept_r is not None:
                y1_l = new_img.shape[0]
                y2_l = int(y1_l-500)
                x1_l = int((y1_l-intercept_l)/slope_l)
                x2_l= int((y2_l-intercept_l)/slope_l)
                y1_r = new_img.shape[0]
                y2_r = int(y1_r-500)
                x1_r = int((y1_r-intercept_r)/slope_r)
                x2_r= int((y2_l-intercept_r)/slope_r)
    
                if (x2_r - x2_l) > 200:
                    cv.line(new_img, (x1_l,y1_l), (x2_l,y2_l), (255,0,0), 10)
                    cv.line(new_img, (x1_r,y1_r), (x2_r,y2_r), (0,0,255), 10)
                    cv.line(new_img, (x1_l, y1_l), (x1_r, y1_r), (0,255,255), 10)
                    cv.line(new_img, (x2_l, y2_l), (x2_r, y2_r), (0,255,255), 10)
    
                    overlay = new_img.copy()
                    output = new_img.copy()
                    pts = np.array([[x1_l,y1_l],[x1_r,y1_r],[x2_r,y2_r],[x2_l,y2_l]], np.int32)
                    
                    #overlay polygon on detected lane
                    cv.fillConvexPoly(overlay, pts, (0,255,0))
                    cv.arrowedLine(overlay, (int((x1_l+x1_r)/2), int((y1_l+y1_r)/2)), 
                                   (int((x2_l+x2_r)/2), int((y2_l+y2_r)/2)), [0,0,255], 2 )
                    
                    #detect position of vehicle i.e left, right or center
                    diff = int((x1_l+x1_r)/2) - int(new_width/2)
                    if diff > 3:
#                        print("left")
                        cv.putText(image, 'Left', (600,50), font,1, (0,0,255), 3, cv.LINE_AA) 
                    elif diff < -3:
#                        print("right")
                        cv.putText(image, 'Right', (600,50), font,1, (0,0,255), 3, cv.LINE_AA) 
                    else:
                        cv.putText(image, 'Center', (600,50), font,1, (0,0,255), 3, cv.LINE_AA)
                    
                    if (int((x1_l+x1_r)/2) - int((x2_l+x2_r)/2)) != 0:
                        slope_center = (int((y1_l+y1_r)/2) - int((y2_l+y2_r)/2))/(int((x1_l+x1_r)/2) - int((x2_l+x2_r)/2))
                   
                    cv.addWeighted(overlay, alpha, output, 1 - alpha,0, output)
                    new_warp = warp(h_inv,ref_frame,image,overlay)
    
                    cv.imshow("overlay", output)
                    
                    #perform inverse warping to put image back on main frame
                    inv_warp_final = warp(h_inv, ref_frame, image, output )
                      
                    cv.imshow("new warp", inv_warp_final)
                    
        else:
            pass
        
        k = cv.waitKey(20) & 0xFF
        if k == 27:
            break
    else:
        break

vid.release()
 
cv.destroyAllWindows()
    
