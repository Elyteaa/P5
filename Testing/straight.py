#test for going in a straight line

import time	
from easygopigo3 import EasyGoPiGo3

gpg = EasyGoPiGo3()

gpg.drive_cm(50, True)
time.sleep(1)