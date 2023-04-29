import cv2
import numpy as np

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
	non_track_ratio = 0.4 # TODO Needs actual ratio from robot
	height=len(img)
	width=len(img[0])
	cv2.rectangle(img, (0, 0), (width, int(height * non_track_ratio)), (0, 0, 0), -1)
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
	kernel = np.ones((4,4),np.uint8)

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

	lines = cv2.HoughLinesP(mask, 1, np.pi / 180, 50, 10, 20)
			# cv2.HoughLinesP(E,Rres,Thetares,Threshold,minLineLength,maxLineGap)
	dots1 = []
	dots2 = []
	# Iterate over segments
    # Draw the lines
	if lines is not None:
		for i in range(0, len(lines)):
			x1, y1, x2, y2 = lines[i][0]

			dx = x2-x1
			dy = y2-y1
			thres_dist = 75
			dist = (dx**2+dy**2)**.5
			if dx!=0 and dy!=0 and dist>thres_dist and abs(1-abs((dy/dx))) < 5 and abs(1-abs((dx/dy))) < 5: # this is a random tan(theta) threshold lol
				# calculate rho theta of this pair
				m = dy / dx
				theta = np.arctan(m)
				rho = x1 * np.cos(theta) + y1 * np.sin(theta)
				thresholded_lines.append((rho,theta))
				
				# separates the line segments into left and right side
				if m<0:
					line1.append((rho,theta))
					dots1.append((x1,y1))
					dots1.append((x2,y2))
					cv2.line(img,(x1,y1), (x2,y2),(0,255,0),2)
				else:
					line2.append((rho,theta))
					dots2.append((x1,y1))
					dots2.append((x2,y2))
					cv2.line(img,(x1,y1), (x2,y2),(0,255,0),2)
	

	rho1 = np.median([x[0] for x in line1])
	theta1 = np.median([x[1] for x in line1]) 
	rho2 = np.median([x[0] for x in line2]) 				
	theta2 = np.median([x[1] for x in line2]) 
	two_lines = [(theta1, rho1), (theta2, rho2)]
	print(two_lines)
		# # Maintain a simples lookup list for points
		# lines_list.append([(x1,y1),(x2,y2)])
	dot1 = dots1[0]
	dot2 = dots2[0]

	for dot in dots1:
		if dot[0]>dot1[0]:
			dot1=dot
	for dot in dots2:
		if dot[0]<dot2[0]:
			dot2=dot
	# dot1 = (max(x[0] for x in dots1), min(x[1] for x in dots1))
	# dot2 = (min(x[0] for x in dots2), min(x[1] for x in dots2))
	goal = (int((dot1[0]+dot2[0])/2), int((dot1[1]+dot2[1])/2))
	# cv2.circle(img, dot1, 5, (255,0,0), 3)
	# cv2.circle(img, dot2, 5, (255,0,0), 3)
	cv2.circle(img, goal, 10, (255, 0, 255), 3)

	# Save the result image
	cv2.imwrite('detectedLines.png',img)

	return goal


# file = ".\johnson\IMG_20230426_170554291.jpg"
# file = ".\johnson\IMG_20230426_170926916.jpg"
# file = ".\johnson\IMG_20230426_170637041.jpg" # side straight
file = ".\johnson\IMG_20230426_171050506.jpg" #curve
image = cv2.imread(file)
if image is None:
     print("Check file path")
    
# for smaller visualization
image = cv2.pyrDown(image)

# print(image)
cd_color_segmentation(image)
