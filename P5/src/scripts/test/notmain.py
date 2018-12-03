#!/usr/bin/python
import rospy
class numbers:
    def __init__(self, one):
        self.one = one
        
    def numberss(self):
        check = True
        self.one = 0
        while check:
            for n in range(1, 4):
                for k in range(2, 4):
                    #self.one = (n, k)
                    self.one = n + k
                    print(self.one)
                    if n == 3 and k == 3:
                        check = False
                #print(self.one)
