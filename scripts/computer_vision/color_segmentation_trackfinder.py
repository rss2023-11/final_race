import cv2
import numpy as np
import math
import pdb
from scipy import signal

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
	# theta: The resolution of the parameter θ in radians. We use 1 degree (CV_PI/180)
	# threshold: The minimum number of intersections to "*detect*" a line
	# minLineLength: The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
	# maxLineGap: The maximum gap between two points to be considered in the same line.

	lines = cv2.HoughLinesP(mask, 1, np.pi / 180, 50, 10, 20)
			# cv2.HoughLinesP(E,Rres,Thetares,Threshold,minLineLength,maxLineGap)
	
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

				cv2.line(image,(x1,y1), (x2,y2),(0,255,0),2)

				# calculate rho theta of this pair
				m = dy / dx
				theta = np.arctan(m)
				rho = x1 * np.cos(theta) + y1 * np.sin(theta)
				thresholded_lines.append((rho,theta))

				if m<0:
					line1.append((rho,theta))
				else:
					line2.append((rho,theta))
	
	rho1 = np.median([x[0] for x in line1])
	theta1 = np.median([x[1] for x in line1]) 
	rho2 = np.median([x[0] for x in line2]) 				
	theta2 = np.median([x[1] for x in line2]) 
	two_lines = [(theta1, rho1), (theta2, rho2)]
	print(two_lines)
		# # Maintain a simples lookup list for points
		# lines_list.append([(x1,y1),(x2,y2)])

	# hough_space = np.zeros((180, mask.shape[1]))
	# for line in thresholded_lines:
	# 	rho, theta = line
	# 	for x in range(mask.shape[1]):
	# 		y = int((rho - x*np.cos(theta)) / np.sin(theta))
	# 		if y >= 0 and y < mask.shape[0]:
	# 			hough_space[int(np.degrees(theta)), x] += mask[y, x]

	# peaks, _ = signal.find_peaks(hough_space.flatten())
	# # https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.find_peaks.html

	# # Extract the parameters for the two peaks with the highest values
	# peak_values = hough_space.flatten()[peaks]
	# peak_indices = peak_values.argsort()[-2:][::-1]
	# peak_thetas, peak_rhos = np.unravel_index(peaks[peak_indices], hough_space.shape)
	# print(peak_thetas, peak_rhos)
	# Draw the two lines on the original image
	# for theta, rho in zip(peak_thetas, peak_rhos):
	first = True
	for theta, rho in two_lines:
		a = np.cos(theta) #x
		b = np.sin(theta) #y
		x0 = a*rho
		y0 = b*rho
		x1 = 0
		y1 = int(height - rho/a)
		x2 = int(rho/b)
		y2 = height

		# x1 = int(x0 + 2000*(-b))
		# y1 = int(height - (y0+2000*a))
		# x2 = int(x0 - 2000*(-b))
		# y2 = int(height - (y0-2000*a))
		if first:
			cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 2) # blue
			first = False
		else:
			cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2) # red

	# Save the result image
	cv2.imwrite('detectedLines.png',image)




file = ".\johnson\IMG_20230426_170554291.jpg"
file = ".\johnson\IMG_20230426_170926916.jpg"
# file = ".\johnson\IMG_20230426_170637041.jpg" # side straight
file = ".\johnson\IMG_20230426_171050506.jpg" #curve
image = cv2.imread(file)
if image is None:
     print("Check file path")
    
# for smaller visualization
image = cv2.pyrDown(image)

# print(image)
cd_color_segmentation(image)
