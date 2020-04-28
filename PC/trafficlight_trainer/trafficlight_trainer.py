import dlib
import cv2
import os
import sys
import xml.etree.ElementTree as pars

dir = sys.path[0]

image = []
images = []
annotations = []

img_name_list = os.listdir(dir + "\\images")
print(img_name_list)

for filename in img_name_list:
	image = cv2.imread(dir + "\\images\\" + filename)
	image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
	only_filename = filename.split('.')[0]
	print(only_filename)

	e = pars.parse(dir + "\\annotations\\" + only_filename + ".xml")
	root = e.getroot()
	for bndbox_object in root.findall("object"):
		bndbox_object = root.find("object")
		bndbox_object = bndbox_object.find("bndbox")
		x_min = int(bndbox_object.find("xmin").text)
		y_min = int(bndbox_object.find("ymin").text)
		x_max = int(bndbox_object.find("xmax").text)
		y_max = int(bndbox_object.find("ymax").text)

		images.append(image)
		annotations.append([dlib.rectangle(left=x_min, top=y_min, right=x_max, bottom=y_max)])

options = dlib.simple_object_detector_training_options()
options.be_verbose = True
detector = dlib.train_simple_object_detector(images, annotations, options)

detector.save("trafficlight_detector.svm")
print("finished")