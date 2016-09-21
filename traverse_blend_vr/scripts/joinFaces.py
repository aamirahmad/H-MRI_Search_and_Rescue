# -*- coding: utf-8 -*-
"""
Created on Thu Apr 30 13:20:20 2015

@author: eruff

"""

try:
    from time import *
except ImportError as err:
    print("Could not find time module")
    raise err


import bpy



def _GetEdgeData(faces):
  """Find edges from faces, and some lookup dictionaries.

  Args:
    faces: list of list of int - each a closed CCW polygon of vertex indices
  Returns:
    (list of ((int, int), int), dict{ int->list of int}) -
      list elements are ((startv, endv), face index)
      dict maps vertices to edge indices
  """

  edges = []
  vtoe = dict()
  for findex, f in enumerate(faces):
    nf = len(f)
    for i, v in enumerate(f):
      endv = f[(i+1) % nf]
      edges.append(((v, endv), findex))
      eindex = len(edges)-1
      if v in vtoe:
        vtoe[v].append(eindex)
      else:
        vtoe[v] = [ eindex ]
  return (edges, vtoe)


def _GetFaceGraph(faces, edges, vtoe):
  """Find the face adjacency graph.

  Faces are adjacent if they share an edge,
  and the shared edge goes in the reverse direction.
  a
  Args:
    faces: list of list of int
    edges: list of ((int, int), int) - see _GetEdgeData
    vtoe: dict{ int->list of int } - see _GetEdgeData
  Returns:
    (list of  list of int, list of bool) -
      first list: each sublist is adjacent face indices for each face
      second list: maps edge index to True if it separates adjacent faces
  """

  face_adj = [ [] for i in range(len(faces)) ]
  is_interior_edge = [ False ] * len(edges)
  for e, ((vs, ve), f) in enumerate(edges):
    for othere in vtoe[ve]:
      ((_, we), g) = edges[othere]
      if we == vs:
        # face g is adjacent to face f
        if g not in face_adj[f]:
            if len(face_adj[f]) > 0:
                if _checkNormals(face_adj[f][0],g):
                    face_adj[f].append(g)
                    is_interior_edge[e] = True
                    # Don't bother with mirror relations, will catch later
            else:
                face_adj[f].append(g)
                is_interior_edge[e] = True
                # Don't bother with mirror relations, will catch later
                    
  return (face_adj, is_interior_edge)

def _checkNormals(a,b):
    obj = bpy.context.active_object
    obj.data.polygons[a].normal
    for x,y in zip(obj.data.polygons[a].normal,obj.data.polygons[b].normal):
         if abs(x) != abs(y):
             return False
    return True
    
def _FindFaceGraphComponents(faces, face_adj):
  """Partition faces into connected components.

  Args:
    faces: list of list of int
    face_adj: list of list of int - see _GetFaceGraph
  Returns:
    (list of list of int, list of int) -
      first list partitions face indices into separate lists, each a component
      second list maps face indices into their component index
  """

  if not faces:
    return ([], [])
  components = []
  ftoc = [ -1 ] * len(faces)
  for i in range(len(faces)):
    if ftoc[i] == -1:
      compi = len(components)
      comp = []
      _FFGCSearch(i, faces, face_adj, ftoc, compi, comp)
      components.append(comp)
  return (components, ftoc)


def _FFGCSearch(findex, faces, face_adj, ftoc, compi, comp):
  """Depth first search helper function for _FindFaceGraphComponents

  Searches recursively through all faces connected to findex, adding
  each face found to comp and setting ftoc for that face to compi.
  """

  comp.append(findex)
  ftoc[findex] = compi
  for otherf in face_adj[findex]:
    if ftoc[otherf] == -1:
      _FFGCSearch(otherf, faces, face_adj, ftoc, compi, comp)
      
def _GetCoplanarJoiningFaces(face_adj):

    for fa in face_adj:
        for f in fa:        
            for vert in current_obj.data.polygons[f].vertices[:]:
                print("vert", vert, " vert co", current_obj.data.vertices[vert].co[2])
            
                print("polygon faces ", )    
#*******************************************************
#
# start the main computation block
#
#*******************************************************
      

      
current_obj = bpy.context.active_object

start_time = time()
print("start",strftime("%Y-%m-%d %H:%M:%S",gmtime(start_time)))


print("="*40) # printing marker
faces = []
for idx,polygon in enumerate(current_obj.data.polygons):
    faces.append(list(polygon.vertices[:]))

    print(polygon.edge_keys," ",idx)
"""

finding_faces_time = time()-start_time
print("finding faces ",finding_faces_time)

(edges, vtoe) = _GetEdgeData(faces)
getEdgeData_time = time()-start_time -finding_faces_time

print(edges[:10]," len edges ",len(edges))
print("vtoe ",vtoe[10])
print("from start Get edge data ",getEdgeData_time)



(face_adj, is_interior_edge) = _GetFaceGraph(faces, edges, vtoe)
getFraceGraph_time = time()-start_time - finding_faces_time - getEdgeData_time
print("from start Get face graph ",getFraceGraph_time)
print(face_adj[:10]," is interior edge ",is_interior_edge[:10])

max_len = 0
retfadj = []
for fadj in face_adj:
    if len(fadj) > max_len:
        max_len = len(fadj)
        retfadj = fadj
        
print(retfadj)

# deselect everything
bpy.ops.object.mode_set(mode = 'EDIT')
bpy.ops.mesh.select_all(action = 'DESELECT')
# reselect the originally selected face
bpy.ops.object.mode_set(mode = 'OBJECT')
# reselect the originally selected face
for i in retfadj:
    
    current_obj.data.polygons[i].select = True
    print(current_obj.data.polygons[i].normal)

bpy.ops.object.mode_set(mode = 'EDIT')    
"""
"""
(components, ftoc) = _FindFaceGraphComponents(faces, face_adj)
findFaceGraphComponents_time = time()-start_time - finding_faces_time - getEdgeData_time- getFraceGraph_time
print("from start Get face graph ",findFaceGraphComponents_time)

"""

print("total run ",time()-start_time)
print("done")