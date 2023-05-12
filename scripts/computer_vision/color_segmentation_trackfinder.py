import cv2
import numpy as np
from scipy import stats
import rospy

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("Bounding Box", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########
    
	# only keep tracks visible
	non_track_ratio = 0.43 # TODO Needs actual ratio from robot
	height=len(img)
	width=len(img[0])
	cv2.rectangle(img, (0, 0), (width, int(height * non_track_ratio)), (0, 0, 0), -1)
	# cv2.rectangle(img, (0, int(height * 0.7)), (width, height), (0, 0, 0), -1)
	hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	# image_print(hsv)

	# lower bound and upper bound for white color
	sensitivity = 80
	lower_bound = np.array([0,0,255-sensitivity])
	upper_bound = np.array([255,sensitivity,255])

	# find the colors within the boundaries
	mask = cv2.inRange(hsv, lower_bound, upper_bound)
	# image_print(mask)

	#define kernel size  
	kernel = np.ones((2,2),np.uint8)

	# Remove unnecessary noise from mask

	mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
	mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	# image_print(mask)
	# Apply HoughLinesP method to
	# to directly obtain line end points
	line1 = []
	line2 = []
	thresholded_lines = []

	# 	with the arguments:
	# lines: A vector that will store the parameters (xstart,ystart,xend,yend) of the detected lines

	# dst: Output of the edge detector. It should be a grayscale image (although in fact it is a binary one)
	# rho : The resolution of the parameter r in pixels. We use 1 pixel.
	# theta: The resolution of the parameter theta in radians. We use 1 degree (CV_PI/180)
	# threshold: The minimum number of intersections to "*detect*" a line
	# minLineLength: The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
	# maxLineGap: The maximum gap between two points to be considered in the same line.

	lines = cv2.HoughLinesP(mask, 2, np.pi / 400, 50, 5, 20)
			# cv2.HoughLinesP(E,Rres,Thetares,Threshold,minLineLength,maxLineGap

	
	dots1 = []
	dots2 = []
	# Iterate over segments
    # Draw the lines
        # rospy.loginfo(lines)
	if lines is not None:
		for i in range(0, len(lines)):
			x1, y1, x2, y2 = lines[i][0]

			dx = x2-x1
			dy = y2-y1
			thres_dist = 75
			dist = (dx**2+dy**2)**.5
			if dx!=0 and dy!=0 and dist>thres_dist and abs(dy/dx)<5 and abs(dx/dy)<5:
				# calculate rho theta of this pair
				m = dy / (1.0 * dx)
				theta = np.arctan2(dy, dx)
				rho = x1 * np.cos(theta) + y1 * np.sin(theta)
				thresholded_lines.append((rho,theta))
				
				# separates the line segments into left and right side
				if m<0:
					line1.append((rho,theta))
					dots1.append([x1,y1])
					dots1.append([x2,y2])
					cv2.line(img,(x1,y1), (x2,y2),(0,255,0),2)
				else:
					line2.append((rho,theta))
					dots2.append([x1,y1])
					dots2.append([x2,y2])
					cv2.line(img,(x1,y1), (x2,y2),(0,255,0),2)
	
	default_turn = int(width/3)
	# default_y = int(height*non_track_ratio+70)
	default_y = int(0.5 * height)
	if not line1 and not line2:
		rospy.logwarn("NOT IN BETWEEN TRACKS")
		return None
	elif not line1: #only see right line, gotta go left
		return (int(width/2) - default_turn, default_y)
	elif not line2: #only seie left line, gotta go right
		return (int(width/2) + default_turn, default_y)
	line1x = [x[0] for x in dots1]
	line1y = [x[1] for x in dots1]
	line2x = [x[0] for x in dots2]
	line2y = [x[1] for x in dots2]
	slope1, intercept1, _,_,_ = stats.linregress(line1x, line1y)
	slope2, intercept2, _,_,_ = stats.linregress(line2x, line2y)
	def getY(x, m, b):
		return (x, int(m*x+b))
	def getX(y, m, b):
		return (int((y-b)/m), y)
	
	# default_x = int((intercept2 - intercept1) / (slope1 - slope2))
	# default_y = int(slope1 * default_x + intercept1)
	cv2.line(img, getY(0, slope1, intercept1), getY(width, slope1, intercept1), (255,0,0), 2)
	cv2.line(img, getY(0, slope2, intercept2), getY(width, slope2, intercept2), (255,0,0), 2)
	x = int((getX(default_y, slope1, intercept1)[0] + getX(default_y, slope2, intercept2)[0])/2)
	# goal = (default_x, default_y)
	goal = (x, default_y)
  cv2.circle(img, goal, 10, (255, 0, 255), 3)
	cv2.imwrite('detectedLines.png',img)
	print(goal)
	return goal

	# # Calculate the angle bisector line Ax + By + C = 0
	# A1, B1, C1 = np.cos(theta1), np.sin(theta1), -rho1
	# A2, B2, C2 = np.cos(theta2), np.sin(theta2), -rho2
	# A, B, C = A1 - A2, B1 - B2, C1 - C2

	# # Second line is the horizontal line y = int(height * non_track_ratio).
	# # Find the intersection point:
	# y = int(height * non_track_ratio)
	# if abs(A) < 0.001:
	# 	rospy.logwarn("Line found is horizontal")
	# 	return None
	
	# x = (-C - B * y) / A



# file = ".\johnson\IMG_20230426_170554291.jpg"
# file = ".\johnson\IMG_20230426_170926916.jpg"
# file = ".\johnson\IMG_20230426_170637041.jpg" # side straight
# file = ".\johnson\IMG_20230426_171050506.jpg" #curve
# image = cv2.imread("final_race/scripts/computer_vision/johnson/IMG_20230426_171054230.jpg")
# image = cv2.imread(file)
# if image is None:
#     print("Check file path")
    
# for smaller visualization
# image = cv2.pyrDown(image)

#print(image)
#cd_color_segmentation(image)
