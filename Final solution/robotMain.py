#!/usr/bin/python
import math
import numpy as np
import time #may be not needed
from robotClass import *

#map1 = WayPoints() #the object that stores all way points of a map

#map1.addWayPoint(1, 0)
#map1.addWayPoint(0, 1)

#print(map1.wayPoint[0])

#path = PlanThePath(map1.wayPoint[0], map1.wayPoint[1])
#print(path.W1, path.W2)
path.plan()