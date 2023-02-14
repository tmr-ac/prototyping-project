import cv2

class ShapeDetector:
	def __init__(self):
		pass

	def detect(self, c):
		# define shape, perimeter and approximated polygon
		shape = "unidentified"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)

		# if shape has 3 vertices --> triangle
		if len(approx) == 3:
			shape = "triangle"

		# if shape has 4 vertices --> square / rectangle
		elif len(approx) == 4:
			(x, y, w, h) = cv2.boundingRect(approx)
			ar = w / float(h)

			# separation for square or rectangle
			shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"

		#otherwise, we assume shape is a circle
		else:
			shape = "circle"

		return shape

