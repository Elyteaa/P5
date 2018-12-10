#test for rotating around own axis

import time 
from easygopigo3 import easygopigo3

gpg = EasyGoPiGo3()

#rotating 180 degrees clockwise, add (-) to turn the other way
gpg.turn_degrees(180)