# -*- coding: utf-8 -*-
"""
Created on Fri Oct 30 13:21:29 2015

@author: eruff

"""
import bpy
from time import time,ctime
import numpy as np

'''py_class.py - Python source designed to demonstrate''' 
'''the use of python embedding'''

class Multiply: 
    def __init__(self): 
        self.a = 6 
        self.b = 5 
        
    def multiply(self):
        c = self.a*self.b
        print ('The result of', self.a, 'x', self.b, ':', c)
        return c
        
    def multiply2(self, a=3, b=4):
        c = a*b
        print ('The result of', a, 'x', b, ':', c)
        return c