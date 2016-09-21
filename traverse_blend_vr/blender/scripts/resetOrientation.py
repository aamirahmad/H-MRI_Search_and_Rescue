# -*- coding: utf-8 -*-
"""
Created on Mon Nov 23 15:05:59 2015

@author: eruff

"""

from bge import logic,events
from mathutils import Quaternion,Euler
import math
import wrapPyRift 		
from numpy import pi
		
class ResetLooks():
	def __init__(self, object):
		self.own = object
		self.contr = None
		
		if isCont(object):
			self.contr = object
			self.own = object.owner
			
		self.pyrift = wrapPyRift.getPyRift()
		if not self.pyrift:
			print("PyRift was NOT initialised")
		

	def getOculusOri(self):
		self.pyrift.poll()
		
		oculus_ori = Quaternion((self.pyrift.rotation[0], 
			self.pyrift.rotation[1], 
			self.pyrift.rotation[2], 
			self.pyrift.rotation[3]))
		
		eu = oculus_ori.to_euler() 
		eu = oculus_ori.to_euler()
		
		fix = Euler((-math.pi/2, 0., math.pi/2), 'XYZ')
		ori = Euler((-eu.z, eu.y, -eu.x), 'XYZ')
		ori.rotate(fix)		    
		
		return ori	
		
		
	def getOrientation(self):
		oculusOri = self.getOculusOri()
		
		ownOri = self.own.localOrientation.to_euler()
		print(ownOri,oculusOri)
		print(ownOri.z - (pi/2 - oculusOri.z), "new orientation")
		
	def setOrientation(self):
		oculusOri = self.getOculusOri()
		
		ownOri = self.own.localOrientation.to_euler()
		print(ownOri,oculusOri)

		ownOri.z = ownOri.z - (pi/2 - oculusOri.z)
		self.pyrift.reset()
		self.own.localOrientation = ownOri.to_matrix()
		
	def resetPosition(self):
		self.own.localPosition = (0.,0.,1.0)
		
		
					


def isCont(object):
	if str(object.__class__) == "<class 'SCA_PythonController'>":
		return True
	return False

def checkInit(contr):
	# get the object this script is attached to
	own = contr.owner
	if 'reset_look.core' not in own:
			own['reset_look.core'] = ResetLooks(contr)

	elif not own['reset_look.core'].pyrift:
		own['reset_look.core'] = ResetLooks(contr)

	return own['reset_look.core'].pyrift


def isActiveCam(contr):
	# get the object this script is attached to
	own = contr.owner
	
	childList = own.children

	scene = logic.getCurrentScene()
	if not scene:
		print("not ready")
		# not ready, main reload(blenderapi)
		return False
	
	if own['reset_look.cam_check']:
		# Do not move the camera if the current view is using another camera
		if own != scene.active_camera and (scene.active_camera not in childList):
#			print("not the active camera ",own.name, scene.active_camera)
			return	False
	return True
	
def run(contr):
	# get the object this script is attached to
	own = contr.owner
	
	if not isActiveCam(contr):
		return
		
	# make sure the Object is initialised
	if not checkInit(contr):
		return
		
				
	JUST_ACTIVATED = logic.KX_INPUT_JUST_ACTIVATED
	keyboard = logic.keyboard
	if keyboard.events[events.NKEY] == JUST_ACTIVATED:
			own['reset_look.core'].getOrientation()
	
	if keyboard.events[events.MKEY] == JUST_ACTIVATED:
			own['reset_look.core'].setOrientation()
	
	if keyboard.events[events.RKEY] == JUST_ACTIVATED:
			own['reset_look.core'].resetPosition()
		