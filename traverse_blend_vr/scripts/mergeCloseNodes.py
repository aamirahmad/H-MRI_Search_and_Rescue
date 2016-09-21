import numpy as np
import bpy
from scipy import spatial
import time
from time import sleep,strftime
import sys
import multiprocessing as mp
import numpy as np, ctypes
import os

class Singleton:
    arr = None

def unwrap_self_f(arg, **kwarg):
    return MergeCloseNodes.generateTreeArray(*arg, **kwarg)


class MergeCloseNodes:
    
    vertices = None
    singleton = Singleton()
    
    def __init__(self,meshName = "Cube"):
        
        self.meshName = meshName
        self.DEBUG = False
        self.THREAD_COUNT = 9  # multiprocessing.cpu_count()-1
    
    """
        get the vertices of the first mesh item in the scene
    """
    def getMeshVertices(self):
        for item in bpy.data.objects:
            if item.type == 'MESH' and (item.name.startswith(self.meshName)):
                vertices = item.data.vertices
                break
        return vertices
    
         
    def info(self,title):
        print(title)
        print('module name:', __name__)
        if hasattr(os, 'getppid'):  # only available on Unix
            print('parent process:', os.getppid())
        print('process id:', os.getpid())
    
    
    def shared_zeros(self,n1, n2):
        # create a 2D numpy array which can be then changed in different threads
        shared_array_base = mp.Array(ctypes.c_double, n1 * n2)
        shared_array = np.ctypeslib.as_array(shared_array_base.get_obj())
        shared_array = shared_array.reshape(n1, n2)
        return shared_array
        
      
    # The worker thread pulls an item from the queue and processes it
    def generateTreeArray(self,i):
        for j in range(3):
            MergeCloseNodes.singleton.arr.itemset((i,j),MergeCloseNodes.vertices[i].co[j])
        return i
    
    def run(self):
        
        start = time.time()
        print("/***********************************************************************/")
        print("MergeCloseNodes.run() process",strftime("%Y-%m-%d %H:%M:%S",time.gmtime()))
             
        bpy.ops.object.mode_set(mode = 'OBJECT')
        
        
        MergeCloseNodes.vertices = self.getMeshVertices()
        num_vertices = len(MergeCloseNodes.vertices)
        print("number of vertices",num_vertices)
        MergeCloseNodes.singleton.arr=self.shared_zeros(num_vertices,3)
        
        pool = mp.Pool(processes=self.THREAD_COUNT)
        pool.map(unwrap_self_f,zip([self]*num_vertices,range(num_vertices)))
        pool.close()
        pool.join()
        print("treeArray took %.2f seconds" % (time.time()-start))
        
        
        treeArray = np.array(MergeCloseNodes.singleton.arr)
        if self.DEBUG:
            print(treeArray,"treeArray")
        
        pair_radius = .001
        qPairs = [0]
        # create a kd-tree from a mesh
        threeDTree = spatial.KDTree(treeArray)
        qPairs = list(threeDTree.query_pairs(r=pair_radius))
        if self.DEBUG:
            print("KDTree.query_pairs(",pair_radius,")", qPairs)
            print("total number of Pairs",len(qPairs))
        
        
        
        # deselect everything
        bpy.ops.object.mode_set(mode = 'EDIT')
        bpy.ops.mesh.select_all(action = 'DESELECT')
        
        for ob in bpy.context.scene.objects:
            if ob.type == 'MESH' and (ob.name.startswith("Cube") or (ob.name.startswith("Plane"))):
                if self.DEBUG:
                    print("entering while loop",strftime("%Y-%m-%d %H:%M:%S",time.gmtime()))
                while len(qPairs) > 0:
                    start_while = time.time()
                    bpy.ops.object.mode_set(mode = 'OBJECT')
                    if self.DEBUG:
                        print("start while loop",strftime("%Y-%m-%d %H:%M:%S",time.gmtime()))
                        print("remaining Pairs",len(qPairs))
                    bpy.ops.object.mode_set(mode = 'OBJECT')
                    
                    # reselect the originally selected face
                    
        
                    x = qPairs[0][0]
                    y = qPairs[0][1]
                    qPairs.pop(0) # remove the first item from the list
                    if not (x == y):
                        ob.data.vertices[x].select = True
                        ob.data.vertices[y].select = True
                        bpy.ops.object.mode_set(mode = 'EDIT')
                        bpy.ops.mesh.merge(type='CENTER')
                        bpy.ops.mesh.select_all(action = 'DESELECT')
                        # decrese the indexes
                        if x > y:
        #                    print("x > y: switching")
                            tmp = y
                            y = x
                            x = tmp
                        
                        for j in range(len(qPairs)):
                            qList = list(qPairs[j])
                            for i in range(len(qList)):
                                if qList[i] == y:                    
                                    qList[i] = x
        #                            print("qList[i] == y",qList[i],"==" ,y)
        #                            print("qList",qList)
                                if qList[i] > y:
                                    qList[i] -= 1
        #                            print("qList[i] > y",qList[i]+1,">" ,y)
        #                            print("qList",qList)
                            qPairs[j] = tuple(qList)
                    if self.DEBUG:
                        print("one while loop tool %.2f seconds" % (time.time()-start_while))
        
        #    bpy.ops.object.mode_set(mode = 'OBJECT')                
        #    qPairs_updated = list(qPairs)
        #    # create a kd-tree from a mesh
        #    threeDTree = spatial.KDTree(getTreeArray())
        #    qPairs_comp = list(threeDTree.query_pairs(r=pair_radius))
        #    
        #    assertion = True
        #    for q in qPairs_updated:
        #        if q not in qPairs_comp:
        #            if tuple(list(q)[::-1]) in qPairs_comp:
        #                print(tuple(list(q)[::-1])),
        #                assertion = True
        #            else:
        #                print("neither",q,"nor",tuple(list(q)[::-1]))
        #                assertion = False
        #            if(list(q)[0] == list(q)[1]):
        #                assertion = True
        #            
        #    if not assertion:    
        #        print("KDTree.query_pairs(",pair_radius,")", qPairs_comp)
        #        print(qPairs_updated)
        #        
        #        break
        
        bpy.ops.object.mode_set(mode = 'EDIT')
        
        verts = self.getMeshVertices()
        print("remaining vertices",len(verts))
#        edges = ob.data.edge_keys
#        print("ob edge keys",len(ob.data.edge_keys))
#        bpy.ops.object.mode_set(mode = 'OBJECT')
#        for i in range(len(verts)):
#            i_edges = [item for item in edges if i in item]
#            i_deg = len(i_edges)
#            if i_deg > 21:
#                print(i,"is in",i_edges,"and has degree",i_deg)
#                ob.data.vertices[i].select = True
        
#        bpy.ops.object.mode_set(mode = 'EDIT')
                            
        #print("Selecting interior faces")
        #bpy.ops.mesh.select_interior_faces()
        
        #print("Deleting interior faces")
        #bpy.ops.mesh.delete(type='ONLY_FACE')
        
        #print("the current time is",strftime("%Y-%m-%d %H:%M:%S",time.gmtime()))
        print("MergeCloseNodes.run() took %.3f seconds" % (time.time()-start))
        print("/***********************************************************************/")
        bpy.ops.object.mode_set(mode = 'OBJECT')
        return True
            