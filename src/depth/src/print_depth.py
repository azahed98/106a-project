#!/usr/bin/env python
import freenect
import rospy
import cv2
import matplotlib.pyplot as plt 
import numpy as np


mdev = freenect.open_device(freenect.init(), 0)
freenect.set_depth_mode(mdev, freenect.RESOLUTION_HIGH, freenect.DEPTH_REGISTERED)
# freenect.runloop(dev=mdev)
def display_depth(dev, data, timestamp):
	d = np.array(data.copy())
	# with open('depth_'+str(timestamp)+'.npy', 'w+') as f:
	# 	np.save(f, d, allow_pickle=True)
	d[d>2000] = np.mean(d[d<2000].flatten())
	plt.imshow(d)
	plt.show()

	# if cv2.WaitKey(10) == 27:
	# 	keep_running = False
# d = freenect.sync_get_depth()[0]
freenect.runloop(dev=mdev, depth=display_depth)
# plt.imshow(d, interpolation='nearest')
# plt.show()
