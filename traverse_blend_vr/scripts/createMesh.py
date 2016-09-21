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


class MeshFromOcTree:
    def __init__(self,ocTree,filename,lvl):
    
        self.__ctx = bpy.context
        self.__scene = self.__ctx.scene
        self.__ocTree = ocTree
        self.saveToFile = filename
        self.saveToFile = '/tmp/octomapFromFile'
        self.__depth = lvl

        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete()
        
    
    def run(self):
        def partition(a, bucket_size):
            buckets = defaultdict(list)
            for n,i in zip(a,range(len(a))):
                buckets[int(i/bucket_size)].append(n)        
            return buckets    
        
        start_time = time()
        (verts,edges,faces) = self.__mapFromData(self.__ocTree,self.__depth)
        print("createMesh mapFromData faces took ",time()-start_time)
        
        start_time = time()
        print("created the mesh data",len(verts),len(edges),len(faces))
        if len(verts) > 8*10:
            #mcn = MergeCloseNodes(meshName="mapFromData")
            print("occupied verticies in ocTree", len(verts)/8)
            # get the path where the blend file is located
            path = bpy.path.abspath('//')
            
            # the smaller this number is the faster the face fill will be
            # but more iterations cost more time
            step = 10
            for i in range(0,int(np.ceil(len(verts)/8/step))):
                start_one_ob = time()
                start_time = time()
                ob = self.__createMeshFromData("mapFromData_"+str(i), (0,0,0), 
                                                 verts[i*step*8:min((i+1)*step*8,len(verts))],
                                                       [[x-i*step*8,y-i*step*8] 
                                                       for x,y in edges[i*step*12:min((i+1)*step*12,len(edges))]],
                                                       [[x-i*step*8,y-i*step*8,z-i*step*8,w-i*step*8] 
                                                       for x,y,z,w in faces[i*step*6:min((i+1)*step*6,len(faces))]])
                                          
#                print("__createMeshFromData ",time()-start_time)
                self.__scene.update()
                ff_start_time = time()
                self.__fillFaces(ob)    
#                print("__fillFaces ",time()-ff_start_time)
                self.__scene.objects.active = ob

#                buckets = partition(self.__scene.objects,1)
#                print('join n select ',len(buckets),'buckets')
#                for j in  buckets:
#                    self.__joinNDeselct(buckets[j])
                
                saveToPath =  self.saveToFile +"_fromData_lvl"+str(self.__depth)+"_"+str(i)+".blend"
#                print("save map to",saveToPath)
                self.__saveTo(saveToPath)
                
                bpy.ops.object.mode_set(mode='OBJECT')
                bpy.ops.object.select_by_type(type = 'MESH')
                bpy.ops.object.delete(use_global=False)
                for item in bpy.data.meshes:
                    bpy.data.meshes.remove(item)
                #ob.select = True
                # save the object to an object file to be imported later
                # make sure that we only export meshes
                '''
                if ob.type == 'MESH':
                    # export the currently selected object to its own file based on its name
                    bpy.ops.export_scene.obj(filepath=str(path + "/object_files/" + ob.name + '.obj'), use_selection=True)
                # deselect the object and move on to another if any more are left
                '''
                #ob.select = False    
#                print("createMesh one object took  ",time()-start_one_ob)



        
        
        # find all meshes and use the edge face add function to generate faces
        # this function sets also the physics in blender game to no collision
        print("join n deselect")
        


        return True
        
    
    def __makeMaterial(self,name, diffuse, specular, alpha):
        mat = bpy.data.materials.new(name)
        mat.diffuse_color = diffuse
        mat.diffuse_shader = 'LAMBERT' 
        mat.diffuse_intensity = 1.0 
        mat.specular_color = specular
        mat.specular_shader = 'COOKTORR'
        mat.specular_intensity = 0.5
        mat.alpha = alpha
        mat.use_shadeless = True
        mat.ambient = 1
        mat.emit = .75
        return mat        
        
    def __createMeshFromData(self,name, origin, verts,edges, faces):

        # Create mesh and object
        me = bpy.data.meshes.new(name+'Mesh')
        ob = bpy.data.objects.new(name, me)
        ob.location = origin
        ob.show_name = True
    
        # Link object to scene and make active
        self.__scene.objects.link(ob)

        self.__scene.objects.active = ob
        ob.select = True
        
        # Create mesh from given verts, faces.
        me.from_pydata(verts, edges, [])
        # Update mesh with new data
        me.update()    
        red = self.__makeMaterial('Red', (.3,.3,.3), (1,1,1), .8)
        me.materials.append(red)
        ob.scale=((2.,2.,2.))
        me.update()    
        self.__scene.update()
        return ob  
        
    def __createMeshFromOperator(self,name, origin, verts,edges, faces):
        bpy.ops.object.add(
            type='MESH', 
            enter_editmode=True,
            location=origin)
        ob = self.__ctx.object
        ob.name = name
        ob.show_name = True
        ob.scale=((2.2,2.2,2.2))
        
        me = ob.data
        me.name = name+'Mesh'
     
        # Create mesh from given verts, faces.
        me.from_pydata(verts, edges, faces)
        # Update mesh with new data
        me.update()    
        # Set object mode
        bpy.ops.object.mode_set(mode='OBJECT')
        return ob  
    """
        fillFaces
        \brief tries to fill the faces of the object provided
        @param ob to object of which the faces should be filled
        @param obName name pattern with which the object needs to begin
    """    
    def __fillFaces(self,ob,obName = "mapFromData"):

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
            ob.game.physics_type = 'NO_COLLISION'
        
    """
        joinNDeselect 
        \brief joins the objects given in the list obs
        if their name is according to meshName (default "Cube")
            
        @param obs list of objects to be joined
        The objects need to be of type MESH        
        
    """
    def __joinNDeselct(self,obs):
        for ob in obs:
            if ob.type == 'MESH':
                ob.select = True
                self.__scene.objects.active = ob
            else:
                ob.select = False
        bpy.ops.object.join()
        bpy.ops.object.select_all(action='DESELECT')           
            
    

    def __mapFromData(self,ocTree,lvl=1,origin=(0,0,0),name="FullMap"):
       
        itr = ocTree.begin_tree()
        num_nodes = ocTree.calcNumNodes()
        print("number of nodes",num_nodes)
        verts = []
        faces = []
        edges = []
        o= 0
        s = 0
        for (n,i) in zip(itr,range(num_nodes)):
            if n.getDepth() == lvl:
                if ocTree.isNodeOccupied(n):
    #                vertsX.append([n.getX()])
    #                vertsY.append([n.getY()])
    #                vertsZ.append([n.getZ()])
                    
                    if n.getZ() > 0.1:
                        x = n.getX()
                        y = n.getY()
                        z = n.getZ()
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
                        
                        faces.append([0+o*8,1+o*8,3+o*8,2+o*8])
                        faces.append([0+o*8,4+o*8,5+o*8,1+o*8])
                        faces.append([0+o*8,2+o*8,6+o*8,4+o*8])
                        faces.append([4+o*8,6+o*8,7+o*8,5+o*8])
                        faces.append([1+o*8,3+o*8,7+o*8,5+o*8])
                        faces.append([2+o*8,6+o*8,7+o*8,3+o*8])
                        
                        edges.append([0+o*8,1+o*8])
                        edges.append([0+o*8,2+o*8])
                        edges.append([0+o*8,4+o*8])
                        edges.append([6+o*8,4+o*8])
                        edges.append([6+o*8,2+o*8])
                        edges.append([6+o*8,7+o*8])                    
                        edges.append([5+o*8,1+o*8])
                        edges.append([5+o*8,4+o*8])
                        edges.append([5+o*8,7+o*8])
                        edges.append([3+o*8,1+o*8])
                        edges.append([3+o*8,2+o*8])
                        edges.append([3+o*8,7+o*8])
                        o+=1
    
        return (np.array(verts),edges,faces)

    def __saveTo(self,filename):
        bpy.ops.wm.save_as_mainfile(filepath=filename)                



    # deprecated part of the class        
    def __createMeshFromPrimitive(self,name, origin,dimension):
        bpy.ops.mesh.primitive_cube_add(
            view_align=False, 
            enter_editmode=False, 
            location=origin, 
            rotation=(0, 0, 0))
     
        ob = self.__ctx.object
        ob.name = name
        ob.scale = dimension
        me = ob.data
        me.name = name
        return ob
     
        
    def __mapFromPrimitives(self,ocTree,octomap_path,lvlDepth):
        global last_joinNDeselect
        global last_save
        global count
        global OCTOMAP_BLEND_FROM_FILE
        
        
        itr = ocTree.begin_tree()
        
        maxDepth = 0
        count = 0
        sizeAtDepth = 0
        lvl = lvlDepth
        start_time = time()
        last_joinNDeselect = time()
        print("start it",start_time)
        for i in itr:        
            if i.getDepth() == lvl:
                if ocTree.isNodeOccupied(i):
                    count += 1
        max_count = count
        print(max_count," nodes at lvl ",lvlDepth)
        count = 0
        itr = ocTree.begin_tree()
        for i in itr:
            #bpy.ops.object.mode_set(mode = 'OBJECT')
            """
                every 200 objects clean up the mess
            """
            if time()-last_save > 60*30:
                print("...save")
                saveTo(OCTOMAP_BLEND_FROM_FILE+"_lvl"+str(lvl)+".blend")
                last_save = time()
                
            if count%200 == 0 and count > 0:
                print(max_count - count," nodes still left to process")
                count += 1
                self.__joinNDeselct()
                mcn = MergeCloseNodes()
                if not mcn.run():
                    raise SystemError
                print("exit cleanup")
                bpy.ops.object.mode_set(mode = 'OBJECT')
            """
                this is how we control the depth and size of the represented map
            """    
            if i.getDepth() == lvl:
                if ocTree.isNodeOccupied(i):
                    count += 1
                    sizeAtDepth = i.getSize()
                    x = i.getSize()/2
                    
                    createMeshFromPrimitive('Cube', (i.getX(),i.getY(),i.getZ()),(x,x,x))
            if maxDepth < i.getDepth():
                maxDepth = i.getDepth()
            
        print("maxDepth",maxDepth,"count", count,"sizeAtDepth",sizeAtDepth)
        
        joinNDeselct()
        mcn = MergeCloseNodes()
        mcn.run()
        saveTo(OCTOMAP_BLEND_FROM_FILE+"_lvl"+str(lvl)+".blend")
        
        print("/*-------------------------------------------------------------------*/")
        print("creating the map took",time()-start_time,"with a max Depth of",lvl)
        print("the start time was",strftime("%Y-%m-%d %H:%M:%S",gmtime(start_time)))
        print("the current time is",strftime("%Y-%m-%d %H:%M:%S",gmtime()))
        print("/*-------------------------------------------------------------------*/")
        