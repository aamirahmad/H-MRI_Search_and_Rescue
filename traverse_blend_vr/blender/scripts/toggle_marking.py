
from bge import logic,events

def removeMarker():
	
	sendMsg = cont.actuators["RemoveMarker"]
	sendMsg.subject = "RemoveMarker"
	sendMsg.body  = str(owner['selected_uav'])
	sendMsg.propName = owner['Selected'].name
	cont.activate(sendMsg)	 
	
	if owner['Selected']["Aktief"]:
		owner['Selected']["Aktief"] = False		

	owner['Selected'] = None
	owner['selected_uav'] = None

if __name__ == "__main__":
	print("enter")
	scene = logic.getCurrentScene()
	cont = logic.getCurrentController()
	owner = cont.owner
	# get active camera
	active_cam = scene.active_camera
	cam_active_change = cont.sensors["cam_active"]

	if cam_active_change.positive:
		print(cam_active_change.bodies)
		if cam_active_change.bodies[0] == 'TRUE':
			owner['mark_survivor'] = False
			owner['select_uav'] = False
			if owner['Selected']:
				removeMarker()
		else:            
			owner['mark_survivor'] = False
			owner['select_uav'] = True
	else:
		keyboard = logic.keyboard
		JUST_ACTIVATED = logic.KX_INPUT_JUST_ACTIVATED
		if keyboard.events[events.RIGHTCTRLKEY] == JUST_ACTIVATED:
			
			if not active_cam['active']:
				owner['mark_survivor'] = not owner['mark_survivor']
				owner['select_uav'] = not owner['mark_survivor']
				print(owner['mark_survivor'],owner['select_uav'])

	sendMsg = cont.actuators["ZendMededeling"]
	sendMsg.subject = "Changed"
	sendMsg.body  = str(owner['select_uav'])
	sendMsg.propName = 'select_uav'
	cont.activate(sendMsg)	 
       
	sendMsg = cont.actuators["Message"]
	sendMsg.subject = "Changed"
	sendMsg.body  = str(owner['mark_survivor'])
	sendMsg.propName = 'mark_survivor'
	cont.activate(sendMsg)	        	