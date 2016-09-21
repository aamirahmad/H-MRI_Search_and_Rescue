from bge import logic

if __name__ == "__main__":
	scene = logic.getCurrentScene()
	cont = logic.getCurrentController()
	owner = cont.owner
	# get active camera
	active_cam = scene.active_camera
	remove_maker = cont.sensors["RemoveMarker"]
	if remove_maker.positive and (str(remove_maker.bodies[0]) == str(owner['uav_id'])):
		cont.activate(cont.actuators["EndMarker"])	 