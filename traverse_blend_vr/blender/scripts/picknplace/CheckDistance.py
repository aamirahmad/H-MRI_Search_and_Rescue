
from bge import logic as GameLogic
import numpy

THRESHOLD = 6.0

def checkDistance():
    if own.name + str("_marker") in scene.objects and not scene.objects["Base"]["Selected"]:
        objPos = own.worldPosition
        markerPos = scene.objects[own.name + str("_marker")].worldPosition
        dist = numpy.linalg.norm(objPos-markerPos)
        if dist < THRESHOLD:
            sendMsg = cont.actuators["Message"]
            sendMsg.subject = "RemoveMarker"
            sendMsg.propName = own.name + str("_marker")
            cont.activate(sendMsg)	


scene = GameLogic.getCurrentScene()
  
    
cont = GameLogic.getCurrentController()
own = cont.owner

if  __name__ == "__main__":
    checkDistance()

	