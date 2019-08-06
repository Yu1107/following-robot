#!/usr/bin/env python
import numpy as np
import rospy
from scipy.misc import imread,imsave


whiteFrame = 255 * np.ones((5000,5000,3), np.uint8)

imsave("Result.png",whiteFrame)
