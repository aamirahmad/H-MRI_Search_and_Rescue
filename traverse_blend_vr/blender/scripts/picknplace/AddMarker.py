
from bge import logic as GameLogic

def addMarker():
    newObject = cont.actuators["AddMarker"]
    newObject.object = own.name + str("_marker")
    newObject.time = 0
    newObject.instantAddObject()

    #cont.activate( newObject )
    print("added Marker")
    
cont = GameLogic.getCurrentController()
scn = GameLogic.getCurrentScene()
own = cont.owner

	
addMarker_msg = cont.sensors["AddMarker"]

if  addMarker_msg.positive :         
    print(int(own['uav_id']) == int(scn.objects['Base']['selected_uav']))
    if int(own['uav_id']) == int(scn.objects['Base']['selected_uav']):
        print(own['uav_id'],scn.objects['Base']['selected_uav'])
        print(own.name,own['uav_id'],"trying to add marker")
        addMarker()
        addedObj  = cont.actuators["AddMarker"].objectLastCreated
        addedObj['uav_id'] = scn.objects['Base']['selected_uav']
        scn.objects['Base']['Selected'] = addedObj
        print (own.name,own['uav_id'], ": add Marker")
        	