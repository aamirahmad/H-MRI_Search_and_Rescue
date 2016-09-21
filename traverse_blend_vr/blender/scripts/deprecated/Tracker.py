from bge import logic

def runTracker():
    # get the current scene
    scene = logic.getCurrentScene()
    # ger the current controller
    # in this case the ground plane
    cont = logic.getCurrentController()
    
    # find the Mouse_over Sensor
    mouse_over = cont.sensors["Mouse_over"]
    
    if mouse_over.positive:
        # get the tracking cursor
        tracker = scene.objects["Tracker"]
        # place the tracking cursor
        tracker.worldPosition = mouse_over.hitPosition
if __name__ == "__main__":
    print("runTracker")
    runTracker()