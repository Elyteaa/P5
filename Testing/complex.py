#test for complex path

import time
from easygopigo3 import EasyGoPiGo3
gpg = EasyGoPiGo3()

gpg.drive_cm(50)
gpg.turn_degrees(90)
gpg.orbit(180, 30)
gpg.turn_degrees(90)
gpg.drive_cm(50)
gpg.turn_degrees(180)