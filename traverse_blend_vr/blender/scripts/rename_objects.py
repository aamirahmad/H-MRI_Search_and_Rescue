# -*- coding: utf-8 -*-
"""
Created on Wed Dec 30 15:36:06 2015

@author: eruff

"""

import bpy

def main(lookfor,replace):
    for obs in bpy.data.objects:
        if obs.type == "MESH":
            if obs.name.startswith(lookfor):
                obs.name = replace
    return
    
    
if __name__ == "__main__":
    print("the main function")