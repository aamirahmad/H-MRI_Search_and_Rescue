
from bge import logic 
import bpy

import sys
sys.path.append("/usr/local/lib/python3.4/dist-packages/rospkg-1.0.35-py3.4.egg")
sys.path.append("/usr/local/lib/python3.4/dist-packages/catkin_pkg")

import datetime

try:
    import roslib
except ImportError as error:
    raise error
import rospy

from geometry_msgs.msg import Pose

cont = logic.getCurrentController()
scn = logic.getCurrentScene()
own = cont.owner

scene = logic.getCurrentScene()

def isInScene(obj_name):
	for objs in logic.getCurrentScene():
		if objs.name == obj.name:
			return True
	return False

def cleanup():
	if own['Selected']["Aktief"]:
		own['Selected']["Aktief"] = False		

	own['Selected'] = None
	own['selected_uav'] = None
	
	
if __name__ == "__main__": 
	# left and right mouse button
	LMB = cont.sensors["LMB"]
	RMB = cont.sensors["RMB"]

	# get active camera
	active_cam = scene.active_camera

	mark_survivor_mode = own['mark_survivor']
	select_uav_mode = own['select_uav']
	if not scn.objects['Desk']['game']:
		print("game is not running") 
	elif active_cam['active'] == True:
		print("deactivate mousemove in order to be able to pick and place")
	else:
			
		if LMB.positive and own['op_mode'] == 'full_autonomous':
			print("in op_mode ",own['op_mode']," no selection possible")
		# in full autonomous mode there is no selection possible
			# if LMB is pressed were in mark survivor mode or in uav select mode but not in full control mode
		if LMB.positive and (mark_survivor_mode or (not (own['op_mode'] == 'full_autonomous') and select_uav_mode)):
	        
			# if something is selected
			if own['Selected']:
				print("deselect'n publish")
				# sent target position to ros
				print(own['Selected'])
				targetPose = own['Selected'].worldPosition
					
				pose = Pose()
				# transform into tk frame
				pose.position.x = 0.5*targetPose[0]
				pose.position.y = 0.5*-targetPose[1]
				pose.position.z = -2
				
				if select_uav_mode:
					scene.objects['Base']['ros_pub_'+ str(own['Selected']['uav_id'])].publish(pose)		
				elif mark_survivor_mode:
					# wirte the scaled pose to the logfile
					with open(own['log_file'], "a") as myfile:
						print("write to logfile",own['log_file'],active_cam.position,str(active_cam.orientation).replace("\n","").replace("  ",""))
						log_str = str(datetime.datetime.now())+ "," \
						+ str(pose.position.x) + "," + str(pose.position.y) +","+ str(active_cam.position) +","+ str(active_cam.orientation).replace("\n","").replace("  ","") + str("\n")
						myfile.write(log_str)			


				cleanup()
			else:
				scene = logic.getCurrentScene()
				objects_on_2nd_layer = [ob.name for ob in bpy.context.scene.objects if ob.layers[1]]

				if select_uav_mode:
					# Op welk object werd geklikt?
					MouseOver = cont.sensors["MouseOver"]
					SelectedObject = MouseOver.hitObject
					
					print("mouseover hitObject Name",MouseOver.hitObject.name,"hitObject uav_id",MouseOver.hitObject['uav_id'])
					# get the hidden objecst on the second layer
							
					if str(SelectedObject) + str("_marker") not in objects_on_2nd_layer:
						print("not a selectable object")
					else:
						# Sla de naam van dit object op
						own['Selected'] = str(SelectedObject) + str("_marker")
						if own['op_mode'] == "semi_autonomous":
							own['selected_uav'] = 1 # set to master
						else:	
							own['selected_uav'] = MouseOver.hitObject['uav_id']
		
						#if not in scene list 
				if mark_survivor_mode:
					print("select survivor")
					SelectedObject = scn.objects['survivor']
					own['Selected'] = str(SelectedObject.name) + str("_marker")
					SelectedObject['uav_id'] -= 1
					own['selected_uav'] = SelectedObject['uav_id']

				#if own['Selected'] in scene.objects scn.objects[:
				alreadyInScene = False

				if own['Selected'] in scn.objects:
					# if the marker is already in scene set the property to the object rather then a string
					for obj in scn.objects:
						if own['Selected'] == obj.name:
							if obj['uav_id'] == own['selected_uav']:
								alreadyInScene = True
								own['Selected'] = obj
								break
				print(alreadyInScene,own["Selected"])
				if own['Selected'] in objects_on_2nd_layer and ((not alreadyInScene) or mark_survivor_mode):
					sendMsg = cont.actuators["ZendMededeling"]
					sendMsg.subject = "AddMarker"
					sendMsg.propName = SelectedObject.name
					cont.activate(sendMsg)		
				else:
					print("send message failed")
					print("object is not in hidden layer or ",own['Selected'], "is in scene objects")
					#cleanup()
				
				own['s'] = own['s'] + 1
				own['Draging'] = True
