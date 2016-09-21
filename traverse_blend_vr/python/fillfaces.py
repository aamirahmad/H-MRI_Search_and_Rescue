import bpy
from time import *

bpy.ops.object.mode_set(mode = 'OBJECT')

overall_start_time = time()
for ob in bpy.context.scene.objects:
    if ob.type == 'MESH' and ob.name.startswith("mapFromData"):
        bpy.context.scene.objects.active = ob   
        ob.select = True
        bpy.ops.object.mode_set(mode = 'EDIT')
        bpy.ops.mesh.select_all(action = 'SELECT')
        try:
            bpy.ops.mesh.edge_face_add()
        except RuntimeError as e:
            print(e)
        ob.select = False

        bpy.ops.mesh.select_all(action = 'DESELECT')
        bpy.ops.object.mode_set(mode = 'OBJECT')
        ob.game.physics_type = 'NO_COLLISION'

print("--------------------> Face Fill took ",time()-overall_start_time," overall")                    
print("done")