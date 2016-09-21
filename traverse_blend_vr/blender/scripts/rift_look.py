
from bge import logic
from mathutils import Quaternion,Euler

import wrapPyRift 
import math

import bpy

from numpy import pi
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
             

from time import time
    
class RiftLook():
	def __init__(self, object):
		
		self.own = object
		self.contr = None
		
		if isCont(object):
			self.contr = object
			self.own = object.owner

			
		self.pyrift = wrapPyRift.getPyRift()

		self.verticalRotation = self.own.localOrientation.to_euler().x * (180 / pi)
		print("========================== %s: init rift look done" % self.own.name)
		


	def getOculusOri(self):
		try:
			self.pyrift.poll()
		except:
			return
		
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

     

def isCont(object):
	if str(object.__class__) == "<class 'SCA_PythonController'>":
		return True
	return False
	
def run(contr):
	# get the object this script is attached to
	own = contr.owner
	
	childList = own.children

	scene = logic.getCurrentScene()
	if not scene:
		print("not ready")
		# not ready, main reload(blenderapi)
		return
	
	if own['rift_look.cam_check']:
		# Do not move the camera if the current view is using another camera
		if own != scene.active_camera and (scene.active_camera not in childList):
#			print("not the active camera ",own.name, scene.active_camera)
			return

	if 'rift_look.core' not in own:
		rl = RiftLook(contr)
		if rl.pyrift:
			own['rift_look.core'] = rl
		else:
			return
	else:
		return own['rift_look.core'].getOculusOri()
	


def main(contr):
	own = contr.owner
	ori = own.localOrientation.to_euler()
	ori = run(contr)
	if ori is not None:
#		print(ori,own.name)
		own.localOrientation = ori.to_matrix()
	

if __name__ == "__main__":
	print("rift look called as script")
	contr = logic.getCurrentController()
	own = contr.owner
		    
		    	
	horizontal = 6
	vertical = 1
		    
		    			
	ori = own.localOrientation.to_euler()
	#ori.x = self.verticalRotation / (180 / math.pi)
		    		
	ori.x += horizontal / (180 / math.pi)
	#ori.y += vertical / (180 / math.pi)
		    
	own.localOrientation = ori.to_matrix()
	print("manuel cam rot ",own.localOrientation.to_euler())			
	
	main(contr)