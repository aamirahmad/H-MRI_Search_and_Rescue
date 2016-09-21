
from bge import logic

if __name__ == "__main__":
    scene = logic.getCurrentScene()
    cont = logic.getCurrentController()
    owner = cont.owner
    toggle_msg = cont.sensors["toggle_mesh"]
    if toggle_msg.positive:
        print(owner.name,toggle_msg.bodies)
        if toggle_msg.bodies[0] == "True":
            mesh_changer = cont.actuators["mesh_active"]
        else:
            mesh_changer = cont.actuators["mesh"]
        cont.activate( mesh_changer )
        
