# -*- coding: utf-8 -*-
"""
Created on Mon Nov 23 15:57:30 2015

@author: eruff

"""
import bpy,bge

try:
	from rift import PyRift
except ImportError:
	import sys
	sys.path.append("/usr/local/lib/python3.4/dist-packages/") 
	try:
		from rift import PyRift
	except ImportError as err:
		print(err)
		raise err

			
DESK_NAME = "Desk"				
PROP_NAME = "PyRift.core"

def init(contr):
	own = contr.owner
	if PROP_NAME not in own:
		print("========================== init PyRift ========================== ")
		own[PROP_NAME] = PyRift()
		


def getPyRift():
	print("========================== get PyRift ========================== ")
	# get the scene
	scene = bge.logic.getCurrentScene()
	# get the object by name
	obj = scene.objects[DESK_NAME]

	if obj != None:
		
		if PROP_NAME in obj:
			return obj[PROP_NAME]
		else:
			print("PyRift is not yet initialised")
			return False
	else:
		print("%s does not exist." % DESK_NAME)
		raise SystemError
             
