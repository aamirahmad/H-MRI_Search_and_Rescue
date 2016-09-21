import bpy


if __name__ == "__main__":
    #find the scene and object
    for scene in bpy.data.scenes:
        print(scene.name)
        distractor_count = 0
        for obj in scene.objects:
            if obj.type == 'MESH' and obj.name.lower().startswith("barrel"):
                filename = "/tmp/barrels_in_" + str(scene.name)
                with open(filename, 'a') as f:
                    f.write(str(obj.name)+","+str(obj.location.x)+","+str(obj.location.y)+'\n')
                print(obj.name,obj.location.x,obj.location.x)
            if obj.type == 'MESH' and obj.name.lower().startswith("cube_"):
                distractor_count+=1
                filename = "/tmp/distractors_in_" + str(scene.name)
                with open(filename, 'a') as f:
                    f.write(str(obj.name)+","+str(obj.location.x)+","+str(obj.location.y)+'\n')
                print(obj.name,obj.location.x,obj.location.x)                
        print("distractor count",distractor_count)
                