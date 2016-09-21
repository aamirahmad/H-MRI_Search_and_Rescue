# -*- coding: utf-8 -*-
"""
Created on Mon May 11 12:22:41 2015

@author: eruff

"""
import bpy
from time import time,strftime ,gmtime
import numpy as np

#import multiprocessing as mp
#import itertools
#from scipy import spatial
from collections import defaultdict
from mergeCloseNodes import MergeCloseNodes
from create_cubes import createBMesh
import bmesh


import bakery
from createBakery import createBakery


class MeshFromOcTree:
        
    def __init__(self):
    
        # shorten some names
        self.__ctx = bpy.context
        self.__scene = self.__ctx.scene
        # remove all objects from the blend
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete()
        self.__depth = 16
        self.__mat_stone = self.__makeMaterial('stone', (0.2,0.2,0.2), (1.,1.,1.), .5)
        self.__mat_violet = self.__makeMaterial('violet', (0.169,0.032,0.8), (1,1,1), .5)


        
    def ocTreeToBlendFile(self,ocTree,depth,saveToPath="/tmp/createMeshTest"):
        start_createMesh = time()
#        self.__createBmeshCubes(ocTree)
        
        verticies = self.__createMeshData(ocTree,depth)
        print("__createMeshData ",time()-start_createMesh)
#        n = 3000
#        verticies = verticies[:8*n]
        
        self.meshDataToBlendFile(verts=verticies,saveToPath=saveToPath)

        
    def meshDataToBlendFile(self,verts=[],saveToPath="/tmp/createMeshTest"):
        
#        return True
        print('verts shape',verts.shape)
        
        start_meshDataTime = time()
        if(len(verts.shape) != 2):
            verts = verts.reshape(-1,3)
        elif (len(verts.shape) == 2 and verts.shape[1] != 3):
            verts = verts.reshape(-1,3)


        try:        
            # get the real end of np arrays
            len_verts = np.nonzero(verts.any(axis=1))[0][-1]+1
        except:
            print("no non zero verts found")
            return

        
        print("get length takes",time()-start_meshDataTime)
    
        print ("len verts",len_verts)
        deep_copy_start = time()
        pVerts = np.copy(verts[:len_verts])

        # HACK use filter instead because it much faster
        filt = np.array([x for x in pVerts if x[2] > 0.15])
        # there are only verts on the ground  BAD HACK
        if len(filt) == 0:
            return

        print("deep_copy takes ",time()-deep_copy_start)

#        print (pVerts[:8])        

        def partition(a, bucket_size):
            buckets = defaultdict(list)
            for n,i in zip(a,range(len(a))):
                buckets[int(i/bucket_size)].append(n)        
            return buckets    

        
        # if the map has at least 10 nodes or exactly one (testing)        
        if len_verts > 8*0 or len_verts == 8:
            MESHNAME= "mapFromData_"
            print("occupied verticies in ocTree", int(len_verts/8))
            # get the path where the blend file is located
            path = bpy.path.abspath('//')
            
            # the smaller this number is the faster the face fill will be
            # but more iterations cost more time
            step = 1000


            face0 = np.array([[0,1,3,2],[0,4,5,1],[0,2,6,4],[4,6,7,5],[1,3,7,5],[2,6,7,3]],dtype=np.int_)
            edge0 = np.array([[0,1],[0,2],[0,4],[6,4],[6,2],[6,7],[5,1],[5,4],[5,7],[3,1],[3,2],[3,7]],dtype=np.int_)             
            
            ## we only need the relative edges and faces 
            # therefore, a set of edges and faces as large as the step is plenty            
            #init the np arrays with the correct size; empty does not assign any values 
            pFaces = np.empty((step*6,4),dtype=np.int_)
            pEdges = np.empty((step*12,2),dtype=np.int_)
           
            for i in range(int(step)):
                pFaces[i*6:i*6+6] = np.add(face0,(i*8))
                pEdges[i*12:i*12+12] = np.add(edge0,(i*8))       
            
            print("number of objects ",int(np.ceil(len_verts/8/step)))
            start_one_ob = time()
            for i in range(0,int(np.ceil(len_verts/8/step))):
                
                start_time = time()
                
                bucket_size = int(((min((i+1)*step*8,len_verts))-(i*step*8))/8)
                
#                ob = createBMesh(MESHNAME+str(i), (0,0,0), pVerts[i*step*8:min((i+1)*step*8,len_verts)].tolist())
                ob = self.__createMeshFromData(MESHNAME+str(i), (0,0,0), 
                                                   # get the nodes within the correct bucket and always 8 at the time to 
                                                   # to have a full cube at hand
                                                        pVerts[i*step*8:min((i+1)*step*8,len_verts)].tolist(),
                                                        # get the edges, but be careful not to overshoot the edge array
                                                        pEdges[:bucket_size*12].tolist(),
                                                        pFaces[:bucket_size*6].tolist()
                                                        )
                                                        

                                       
                
                print("__createMeshFromData ",time()-start_time)
                self.__scene.update()

#                self.__scene.objects.active = ob                    
                    
              
                print("__fillFaces start")
                ff_start_time = time()
                self.__fillFaces(ob)    
                print("__fillFaces ",time()-ff_start_time)
                
                fd_start_time = time()
                
                ob = self.__remove_doubles(ob)
                
                print("__remove_doubles ",time()-fd_start_time)
                ob.game.physics_type = 'NO_COLLISION'  
#                ob.game.physics_type = 'STATIC'  
#                ob.game.physics_type = 'OCCLUDER'  
            
#            if bpy.ops.object.mode_set.poll():
#                bpy.ops.object.mode_set(mode='EDIT')        
#                bpy.ops.mesh.select_all(action = 'SELECT')
#                bpy.ops.mesh.select_all(action = 'DESELECT')
#                bpy.ops.object.mode_set(mode = 'OBJECT')
#                ob.game.physics_type = 'NO_COLLISION'    
#                bpy.ops.object.mode_set(mode = 'EDIT')
#                buckets = partition(self.__scene.objects,1)
#                print('join n select ',len(buckets),'buckets')
#                for j in  buckets:
#                    self.__joinNDeselct(buckets[j])
                
#                saveToFile =  saveToPath +"_fromData_lvl"+str(self.__depth)+"_"+str(i)+".blend"
#                print("save map to",saveToFile)
#                self.__saveTo(saveToFile)
#                
#                bpy.ops.object.mode_set(mode='OBJECT')
#                bpy.ops.object.select_by_type(type = 'MESH')
#                bpy.ops.object.delete(use_global=False)
#                for item in bpy.data.meshes:
#                    bpy.data.meshes.remove(item)
                #ob.select = True
                # save the object to an object file to be imported later
                # make sure that we only export meshes
                '''
                if ob.type == 'MESH':
                    # export the currently selected object to its own file based on its name
                    bpy.ops.export_scene.obj(filepath=str(path + "/object_files/" + ob.name + '.obj'), use_selection=True)
                # deselect the object and move on to another if any more are left
                    
                '''
                
            print("createMesh all objects took  ",time()-start_one_ob)


            # we need to join all objects we have so far to make the baking process more efficient and clean
            self.joinObjects()
            # deselect all objects
#            for sob in bpy.context.selected_objects:
#                sob.select = False
            self.setOrigins('ORIGIN_CENTER_OF_MASS')

            # then we can bake the material as texture to save resources when displaying
            # we loop once more through the objects, however, since there is only one left ...
            for ob in bpy.context.scene.objects:
                # whatever objects you want to join...
                if ob.type == 'MESH':            
#                    ob.select = True
                    
                    bpy.ops.object.origin_set(type='ORIGIN_CENTER_OF_MASS')

                    self.bakeWarpper(ob)
                    break

            saveToFile =  saveToPath +"_fromData_lvl"+str(self.__depth)+".blend"
            print("save map to",saveToFile)
            self.__saveTo(saveToFile)
            print("---------------------------meshDataToBlendFile took ",time()-start_meshDataTime)
        
        
        # find all meshes and use the edge face add function to generate faces
        # this function sets also the physics in blender game to no collision
#        print("join n deselect")
        


        return True

    # reset the orign to all objects in the scene 
    def setOrigins(self,origin='ORIGIN_CENTER_OF_MASS'):
        # sanity check
        if origin not in ['ORIGIN_CENTER_OF_MASS','ORIGIN_GEOMETRY','GEOMETRY_ORIGIN','ORIGIN_CURSOR']:
            return
            
        for ob in bpy.context.scene.objects:
            bpy.context.scene.objects.active = ob
            bpy.ops.object.origin_set(type=origin)

    def joinObjects(self):
        scene = bpy.context.scene

        obs = []
        for ob in scene.objects:
            # whatever objects you want to join...
            if ob.type == 'MESH':
                obs.append(ob)
        
        ctx = bpy.context.copy()
        
        # one of the objects to join
        ctx['active_object'] = obs[0]
        
        ctx['selected_objects'] = obs
        
        # we need the scene bases as well for joining
        ctx['selected_editable_bases'] = [scene.object_bases[ob.name] for ob in obs]
        
        bpy.ops.object.join(ctx)
        
    def bakeWarpper(self,obj):
    

        bpy.context.scene.objects.active = obj
    
        # deselect all objects
        for ob in bpy.data.objects:
            ob.select = False    
    
        bake_config = {'filepath':'/tmp/textures/',
                       'baked_texture': 'baked_image'+str(time()),
                       'baked_texture_img': 'baked_image'+str(time())+'.png',
                       'texture':'baked_texture_'+str(time()),
                       'material':'baked_material_'+str(time()),
                       'uv_map':'baked_uv_map_'+str(time()),
                       'unwrap':'lightmap_pack'}
              
        # set up the scenery to bake a texture to an object
        print('create the bakery')
        cBakery = createBakery()
        # bake the texture
        bakery.bake(obj,bake_config)
        # remove the scenery again
        print("clean up the bakery")
        del cBakery    



    def clearoutalloldmeshes(self):
        return
       
    """
        @brief tries to fill the faces of the object provided
        @param ob to object of which the faces should be filled
        @param obName name pattern with which the object needs to begin
    """    
    def __fillFaces(self,ob,obName = "mapFromData"):
#        scene = bpy.context.scene
#
#        # force edge selection mode
#        bpy.context.tool_settings.mesh_select_mode = (False, False, True) 					
#        me = ob.data
#
#        scene.objects.unlink(ob)
#
#        if me.is_editmode:
#            # Gain direct access to the mesh
#            bm = bmesh.from_edit_mesh(me)
#        else:
#            # Create a bmesh from mesh
#            # (won't affect mesh, unless explicitly written back)
#            bm = bmesh.new()
#            bm.from_mesh(me)
#
#            
#        bmesh.ops.contextual_create(bm, geom=bm.verts[:])
#        # Finish up, write the bmesh into a new mesh
#        me = bpy.data.meshes.new("Mesh")
#        bm.to_mesh(me)
#        bm.free()            
#        
#        # Add the mesh to the scene
#        obj = bpy.data.objects.new("Object", me)
#        scene.objects.link(obj)
#        
#        # Select and make active
#        scene.objects.active = obj
#        obj.select = True        
            

        if ob.type == 'MESH' and ob.name.startswith(obName):
            ob.select = True
            self.__scene.objects.active = ob   
            
            if bpy.ops.object.mode_set.poll():
                bpy.ops.object.mode_set(mode='EDIT')        
                bpy.ops.mesh.select_all(action = 'SELECT')
            try:
                bpy.ops.mesh.edge_face_add()
            except RuntimeError as e:
                print(e)
            ob.select = False
    
            if bpy.ops.object.mode_set.poll():
                bpy.ops.mesh.select_all(action = 'DESELECT')
                bpy.ops.object.mode_set(mode = 'OBJECT')
    
    def __remove_doubles(self,ob):
        
        scene = bpy.context.scene
        
        # force edge selection mode
        bpy.context.tool_settings.mesh_select_mode = (False, False, True) 					
        me = ob.data

        print("unlink",ob.name)
        scene.objects.unlink(ob)
        if me.is_editmode:
            # Gain direct access to the mesh
            bm = bmesh.from_edit_mesh(me)
        else:
            # Create a bmesh from mesh
            # (won't affect mesh, unless explicitly written back)
            bm = bmesh.new()
            bm.from_mesh(me)

        bmesh.ops.remove_doubles(bm,verts=bm.verts[:],dist=0.001)
        
        
#        bmesh.ops.recalc_face_normals(bm,faces=bm.faces[:])
        # Finish up, write the bmesh into a new mesh
        me = bpy.data.meshes.new("Mesh")
        
        bm.to_mesh(me)
        bm.free()
        
        me.materials.append(self.__mat_stone)
        
        
        # Add the mesh to the scene
        obj = bpy.data.objects.new("Object", me)
        scene.objects.link(obj)
        
        obj.scale=((2.,2.,2.))
        me.update()         
        
        # Select and make active
        scene.objects.active = obj
        obj.select = True        
        return obj

        
    def __makeMaterial(self,name, diffuse, specular, alpha, shadeless = False):
        mat = bpy.data.materials.new(name)
        mat.diffuse_color = diffuse
        mat.diffuse_shader = 'LAMBERT' 
        mat.diffuse_intensity = .8 
        mat.specular_color = specular
        mat.specular_shader = 'COOKTORR'
        mat.specular_intensity = .5
        mat.alpha = alpha
        mat.use_shadeless = shadeless
        mat.ambient = .5
        mat.emit = .5
        return mat        
        
    def __createMeshFromData(self,name, origin, verts,edges, faces):

        # Create mesh and object
        me = bpy.data.meshes.new(name+'Mesh')
        ob = bpy.data.objects.new(name, me)
        ob.location = origin
        ob.show_name = True

#        print (edges[10:20])
        # Link object to scene and make active
        self.__scene.objects.link(ob)

        self.__scene.objects.active = ob
        ob.select = True
        
        # Create mesh from given verts, faces.
        me.from_pydata(verts, edges, [])
        # Update mesh with new data
        me.update()    
        

        ob.scale=((2.,2.,2.))
        me.update()    
        self.__scene.update()
        return ob  
        
   
    
    def __createBmeshCubes(self,ocTree):
        
        # Make a new BMesh
        bm = bmesh.new() 
        
        itr = ocTree.begin_tree()
        num_nodes = ocTree.calcNumNodes()
        print("number of nodes",num_nodes)

     
         
    def __createMeshData(self,ocTree,lvl=1,origin=(0,0,0),name="FullMap"):
       
        itr = ocTree.begin_tree()
        num_nodes = ocTree.calcNumNodes()
        print("number of nodes",num_nodes)
        verts = []

        o= 0
        s = 0
        for (n,i) in zip(itr,range(num_nodes)):
            
#            if n.getDepth() == 10:
                #print("coordinates",i,"  %0.2f %0.2f %0.2f" % (n.getX(),n.getY(),n.getZ()))
                #n.getY() n.getZ())
            
            if n.getDepth() == lvl:
                if ocTree.isNodeOccupied(n):
                    
                    if n.getZ() > 0.1:
                        x = n.getX()
                        y = n.getY()
                        z = n.getZ()-0.1
                        s=n.getSize()
                        
    #                    verts.append(n.getCoordinate())
                        verts.append((x+s/2,y+s/2,z+s/2))
                        verts.append((x+s/2,y+s/2,z-s/2))
                        verts.append((x+s/2,y-s/2,z+s/2))
                        verts.append((x+s/2,y-s/2,z-s/2))
                        verts.append((x-s/2,y+s/2,z+s/2))
                        verts.append((x-s/2,y+s/2,z-s/2))
                        verts.append((x-s/2,y-s/2,z+s/2))
                        verts.append((x-s/2,y-s/2,z-s/2))
                        

                        o+=1
    
        return (np.array(verts,dtype=np.float_))

    def __saveTo(self,filename):
        bpy.ops.wm.save_as_mainfile(filepath=filename)                



     
if __name__ == "__main__":
    mefo = MeshFromOcTree()

#    verts = np.add(verts,2)
    verts = np.array([[-8.25, -8.3, 0.2], [-8.25, -8.3, 0.15000000000000002], [-8.25, -8.350000000000001, 0.2], [-8.25, -8.350000000000001, 0.15000000000000002], [-8.3, -8.3, 0.2], [-8.3, -8.3, 0.15000000000000002], [-8.3, -8.350000000000001, 0.2], [-8.3, -8.350000000000001, 0.15000000000000002], [-8.2, -8.3, 0.2], [-8.2, -8.3, 0.15000000000000002], [-8.2, -8.350000000000001, 0.2], [-8.2, -8.350000000000001, 0.15000000000000002], [-8.25, -8.3, 0.2], [-8.25, -8.3, 0.15000000000000002], [-8.25, -8.350000000000001, 0.2], [-8.25, -8.350000000000001, 0.15000000000000002], [-8.25, -8.25, 0.15], [-8.25, -8.25, 0.1], [-8.25, -8.3, 0.15], [-8.25, -8.3, 0.1], [-8.3, -8.25, 0.15], [-8.3, -8.25, 0.1], [-8.3, -8.3, 0.15], [-8.3, -8.3, 0.1], [-8.25, -8.2, 0.15], [-8.25, -8.2, 0.1], [-8.25, -8.25, 0.15], [-8.25, -8.25, 0.1], [-8.3, -8.2, 0.15], [-8.3, -8.2, 0.1], [-8.3, -8.25, 0.15], [-8.3, -8.25, 0.1], [-8.25, -8.25, 0.2], [-8.25, -8.25, 0.15000000000000002], [-8.25, -8.3, 0.2], [-8.25, -8.3, 0.15000000000000002], [-8.3, -8.25, 0.2], [-8.3, -8.25, 0.15000000000000002], [-8.3, -8.3, 0.2], [-8.3, -8.3, 0.15000000000000002], [-8.2, -8.25, 0.2], [-8.2, -8.25, 0.15000000000000002], [-8.2, -8.3, 0.2], [-8.2, -8.3, 0.15000000000000002], [-8.25, -8.25, 0.2], [-8.25, -8.25, 0.15000000000000002], [-8.25, -8.3, 0.2], [-8.25, -8.3, 0.15000000000000002], [-8.0, -8.3, 0.2], [-8.0, -8.3, 0.15000000000000002], [-8.0, -8.350000000000001, 0.2], [-8.0, -8.350000000000001, 0.15000000000000002], [-8.05, -8.3, 0.2], [-8.05, -8.3, 0.15000000000000002], [-8.05, -8.350000000000001, 0.2], [-8.05, -8.350000000000001, 0.15000000000000002], [-8.05, -8.25, 0.15], [-8.05, -8.25, 0.1], [-8.05, -8.3, 0.15], [-8.05, -8.3, 0.1], [-8.100000000000001, -8.25, 0.15], [-8.100000000000001, -8.25, 0.1], [-8.100000000000001, -8.3, 0.15], [-8.100000000000001, -8.3, 0.1], [-8.05, -8.2, 0.15], [-8.05, -8.2, 0.1], [-8.05, -8.25, 0.15], [-8.05, -8.25, 0.1], [-8.100000000000001, -8.2, 0.15], [-8.100000000000001, -8.2, 0.1], [-8.100000000000001, -8.25, 0.15], [-8.100000000000001, -8.25, 0.1], [-8.0, -8.2, 0.15], [-8.0, -8.2, 0.1], [-8.0, -8.25, 0.15], [-8.0, -8.25, 0.1], [-8.05, -8.2, 0.15], [-8.05, -8.2, 0.1], [-8.05, -8.25, 0.15], [-8.05, -8.25, 0.1], [-8.0, -8.25, 0.2], [-8.0, -8.25, 0.15000000000000002], [-8.0, -8.3, 0.2], [-8.0, -8.3, 0.15000000000000002], [-8.05, -8.25, 0.2], [-8.05, -8.25, 0.15000000000000002], [-8.05, -8.3, 0.2], [-8.05, -8.3, 0.15000000000000002], [-8.25, -8.15, 0.15], [-8.25, -8.15, 0.1], [-8.25, -8.200000000000001, 0.15], [-8.25, -8.200000000000001, 0.1], [-8.3, -8.15, 0.15], [-8.3, -8.15, 0.1], [-8.3, -8.200000000000001, 0.15], [-8.3, -8.200000000000001, 0.1], [-8.25, -8.1, 0.15], [-8.25, -8.1, 0.1], [-8.25, -8.15, 0.15], [-8.25, -8.15, 0.1], [-8.3, -8.1, 0.15], [-8.3, -8.1, 0.1], [-8.3, -8.15, 0.15], [-8.3, -8.15, 0.1], [-8.25, -8.15, 0.2], [-8.25, -8.15, 0.15000000000000002], [-8.25, -8.200000000000001, 0.2], [-8.25, -8.200000000000001, 0.15000000000000002], [-8.3, -8.15, 0.2], [-8.3, -8.15, 0.15000000000000002], [-8.3, -8.200000000000001, 0.2], [-8.3, -8.200000000000001, 0.15000000000000002], [-8.25, -8.1, 0.2], [-8.25, -8.1, 0.15000000000000002], [-8.25, -8.15, 0.2], [-8.25, -8.15, 0.15000000000000002], [-8.3, -8.1, 0.2], [-8.3, -8.1, 0.15000000000000002], [-8.3, -8.15, 0.2], [-8.3, -8.15, 0.15000000000000002], [-8.25, -8.0, 0.15], [-8.25, -8.0, 0.1], [-8.25, -8.05, 0.15], [-8.25, -8.05, 0.1], [-8.3, -8.0, 0.15], [-8.3, -8.0, 0.1], [-8.3, -8.05, 0.15], [-8.3, -8.05, 0.1], [-8.25, -8.0, 0.2], [-8.25, -8.0, 0.15000000000000002], [-8.25, -8.05, 0.2], [-8.25, -8.05, 0.15000000000000002], [-8.3, -8.0, 0.2], [-8.3, -8.0, 0.15000000000000002], [-8.3, -8.05, 0.2], [-8.3, -8.05, 0.15000000000000002], [-8.0, -8.1, 0.2], [-8.0, -8.1, 0.15000000000000002], [-8.0, -8.15, 0.2], [-8.0, -8.15, 0.15000000000000002], [-8.05, -8.1, 0.2], [-8.05, -8.1, 0.15000000000000002], [-8.05, -8.15, 0.2], [-8.05, -8.15, 0.15000000000000002], [-8.05, -8.05, 0.15], [-8.05, -8.05, 0.1], [-8.05, -8.100000000000001, 0.15], [-8.05, -8.100000000000001, 0.1], [-8.100000000000001, -8.05, 0.15], [-8.100000000000001, -8.05, 0.1], [-8.100000000000001, -8.100000000000001, 0.15], [-8.100000000000001, -8.100000000000001, 0.1], [-8.0, -8.05, 0.15], [-8.0, -8.05, 0.1], [-8.0, -8.100000000000001, 0.15], [-8.0, -8.100000000000001, 0.1], [-8.05, -8.05, 0.15], [-8.05, -8.05, 0.1], [-8.05, -8.100000000000001, 0.15], [-8.05, -8.100000000000001, 0.1], [-8.05, -8.0, 0.15], [-8.05, -8.0, 0.1], [-8.05, -8.05, 0.15], [-8.05, -8.05, 0.1], [-8.100000000000001, -8.0, 0.15], [-8.100000000000001, -8.0, 0.1], [-8.100000000000001, -8.05, 0.15], [-8.100000000000001, -8.05, 0.1], [-8.0, -8.05, 0.2], [-8.0, -8.05, 0.15000000000000002], [-8.0, -8.100000000000001, 0.2], [-8.0, -8.100000000000001, 0.15000000000000002], [-8.05, -8.05, 0.2], [-8.05, -8.05, 0.15000000000000002], [-8.05, -8.100000000000001, 0.2], [-8.05, -8.100000000000001, 0.15000000000000002], [-8.25, -8.3, 0.25], [-8.25, -8.3, 0.2], [-8.25, -8.350000000000001, 0.25], [-8.25, -8.350000000000001, 0.2], [-8.3, -8.3, 0.25], [-8.3, -8.3, 0.2], [-8.3, -8.350000000000001, 0.25], [-8.3, -8.350000000000001, 0.2], [-8.2, -8.3, 0.25], [-8.2, -8.3, 0.2], [-8.2, -8.350000000000001, 0.25], [-8.2, -8.350000000000001, 0.2], [-8.25, -8.3, 0.25], [-8.25, -8.3, 0.2], [-8.25, -8.350000000000001, 0.25], [-8.25, -8.350000000000001, 0.2], [-8.25, -8.3, 0.30000000000000004], [-8.25, -8.3, 0.25], [-8.25, -8.350000000000001, 0.30000000000000004], [-8.25, -8.350000000000001, 0.25], [-8.3, -8.3, 0.30000000000000004], [-8.3, -8.3, 0.25], [-8.3, -8.350000000000001, 0.30000000000000004], [-8.3, -8.350000000000001, 0.25], [-8.2, -8.3, 0.30000000000000004], [-8.2, -8.3, 0.25], [-8.2, -8.350000000000001, 0.30000000000000004], [-8.2, -8.350000000000001, 0.25], [-8.25, -8.3, 0.30000000000000004], [-8.25, -8.3, 0.25], [-8.25, -8.350000000000001, 0.30000000000000004], [-8.25, -8.350000000000001, 0.25], [-8.25, -8.25, 0.25], [-8.25, -8.25, 0.2], [-8.25, -8.3, 0.25], [-8.25, -8.3, 0.2], [-8.3, -8.25, 0.25], [-8.3, -8.25, 0.2], [-8.3, -8.3, 0.25], [-8.3, -8.3, 0.2], [-8.25, -8.25, 0.30000000000000004], [-8.25, -8.25, 0.25], [-8.25, -8.3, 0.30000000000000004], [-8.25, -8.3, 0.25], [-8.3, -8.25, 0.30000000000000004], [-8.3, -8.25, 0.25], [-8.3, -8.3, 0.30000000000000004], [-8.3, -8.3, 0.25], [-8.25, -8.2, 0.30000000000000004], [-8.25, -8.2, 0.25], [-8.25, -8.25, 0.30000000000000004], [-8.25, -8.25, 0.25], [-8.3, -8.2, 0.30000000000000004], [-8.3, -8.2, 0.25], [-8.3, -8.25, 0.30000000000000004], [-8.3, -8.25, 0.25], [-8.2, -8.2, 0.30000000000000004], [-8.2, -8.2, 0.25], [-8.2, -8.25, 0.30000000000000004], [-8.2, -8.25, 0.25], [-8.25, -8.2, 0.30000000000000004], [-8.25, -8.2, 0.25], [-8.25, -8.25, 0.30000000000000004], [-8.25, -8.25, 0.25]])
    unit_cube = np.array([[0,0,0],[0,0,1],[0,1,0],[0,1,1],[1,0,0],[1,0,1],[1,1,0],[1,1,1]])
    verts = unit_cube
    for z in range(2):
        for y in range(-2,2):
            for x in range(4):
                verts = np.append(verts,np.add(unit_cube,[x,y*2,z]),0)
    verts = np.multiply(verts,0.9)
    print(verts)
    mefo.meshDataToBlendFile(verts)
