from scipy.spatial import distance as dist
from collections import OrderedDict
import numpy as np
import cv2

class ColorLabeler:
	def __init__(self):
		# initialize color dictionary, color name is key and RGP tuple is value
		colors = OrderedDict({
			"red": (255, 0, 0),
			"green": (0, 255, 0),
			"pink": (255, 0, 255),
			"yellow": (255, 255, 0)})

		# create numpy array and initialze color names list
		self.lab = np.zeros((len(colors), 1, 3), dtype="uint8")
		self.colorNames = []

		# loop over colors dictionary
		for (i, (name, rgb)) in enumerate(colors.items()):
			# update L*a*b array and color names list
			self.lab[i] = rgb
			self.colorNames.append(name)

		# convert L*a*b array from RGB color to L*a*b
		self.lab = cv2.cvtColor(self.lab, cv2.COLOR_RGB2LAB)

	def label(self, image, c):
		# construct mask for contour, then compute average L*a*b value for region
		mask = np.zeros(image.shape[:2], dtype="uint8")
		cv2.drawContours(mask, [c], -1, 255, -1)
		mask = cv2.erode(mask, None, iterations = 2)
		mean = cv2.mean(image, mask=mask)[:3]

		# initialize minimum distance found thus far
		minDist = (np.inf, None)

		# loop over known L*a*b color values
		for (i, row) in enumerate(self.lab):
			#compute distance between current L*a*b color value and mean of image
			d = dist.euclidean(row[0], mean)

			# if distance is smaller than current distance, update bookkeeping var
			if d < minDist[0]:
				minDist = (d, i)

		# return name of color with smallest distance
		return self.colorNames[minDist[1]]


