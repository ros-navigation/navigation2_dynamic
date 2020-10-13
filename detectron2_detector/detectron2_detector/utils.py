import numpy as np 
from nav2_dynamic_msgs.msg import Obstacle
from functools import reduce

def IoU_3D(obstacle1, obstacle2):
	'''computer IoU of 3D bounding box given 2 Obstacle msgs '''
	p1 = [obstacle1.position.x, obstacle1.position.y, obstacle1.position.z]
	s1 = [obstacle1.size.x, obstacle1.size.y, obstacle1.size.z]
	p2 = [obstacle2.position.x, obstacle2.position.y, obstacle2.position.z]
	s2 = [obstacle2.size.x, obstacle2.size.y, obstacle2.size.z]

	max1 = [p1[i] + s1[i] / 2 for i in range(3)]
	min1 = [p1[i] - s1[i] / 2 for i in range(3)]
	max2 = [p2[i] + s2[i] / 2 for i in range(3)]
	min2 = [p2[i] - s2[i] / 2 for i in range(3)]

	overlap = [max(0, min(max1[i], max2[i]) - max(min1[i], min2[i])) for i in range(3)]
	vol = lambda x,y:x*y
	intersection = reduce(vol, overlap)
	union = reduce(vol, s1) + reduce(vol, s2) - intersection
	return intersection / union

def NMS_3D(obstacles, threshold):
	'''use non-max suppression to filter a list of obstacles '''
	if len(obstacles) < 2:
		return obstacles 

	obstacles.sort(reverse = True, key = lambda x: x.score)
	bboxes = [obstacles[0]]
	for i in range(1, len(obstacles)):
		bbox = obstacles[i]
		flag = True
		for j in range(len(bboxes)):
			if IoU_3D(bboxes[j], bbox) > threshold:
				flag = False
				break
		if flag:
			bboxes.append(bbox)
	return bboxes
