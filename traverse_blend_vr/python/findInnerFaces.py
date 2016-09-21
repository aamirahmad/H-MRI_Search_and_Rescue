# -*- coding: utf-8 -*-
"""
Created on Wed Nov 18 18:41:19 2015

@author: eruff

"""

import bpy
import bmesh
from time import time
#ob = bpy.context.object


def getFaceEdges(ob):
	if ob.type != 'MESH':
		raise TypeError("Active object is not a Mesh")

	# Get editmode changes
	ob.update_from_editmode()
	
	me = ob.data
	
	if len(me.polygons) < 1:
		raise ValueError("Mesh has no faces")

	# Build lookup dictionary for edge keys to edges
	edges = me.edges
	face_edge_map = {ek: edges[i] for i, ek in enumerate(me.edge_keys)}
	
	print(face_edge_map)
			
	# Get active face by index
	face = me.polygons[me.polygons.active]
	
	selected = "selected"
	not_selected = " ".join(("NOT", selected))

	for ek in face.edge_keys:
		edge = face_edge_map[ek]
		print (ek)
#		print("%12s - %r" % ((selected if edge.select else not_selected), edge))	

def getFaceEdgesBMESH(ob):
	
	if ob.type != 'MESH':
		raise TypeError("Active object is not a Mesh")

	# force edge selection mode
	bpy.context.tool_settings.mesh_select_mode = (False, False, True) 					
	me = ob.data
	
	if me.is_editmode:
		# Gain direct access to the mesh
		bm = bmesh.from_edit_mesh(me)
	else:
		# Create a bmesh from mesh
		# (won't affect mesh, unless explicitly written back)
		bm = bmesh.new()
		bm.from_mesh(me)

					
#	# Get active face
#	for face in bm.faces:
#		for edge in face.edges:
#			for face1 in bm.faces:
#				for edge1 in face1.eges:
	
#	selected = "selected"
#	not_selected = " ".join(("NOT", selected))
#	
#	
#	for edge in face.edges:
#		print("%12s - bm.edges[%i]" % ((selected if edge.select else not_selected), edge.index))
	
	
	# No need to do anything here if you haven't changed the (b)mesh
	# Otherwise, flush changes from wrapped bmesh / write back to mesh
	"""
	if me.is_editmode:
	    bmesh.update_edit_mesh(me)
	else:
	    bm.to_mesh(me)
	    me.update()
	
	bm.free()
	del bm
	"""


def getFaces(obj):		
	if obj.type != 'MESH':
		raise TypeError("Active object is not a Mesh")
		
	bpy.context.tool_settings.mesh_select_mode = (False, False, True) # face select		
	for face in obj.data.polygons:
		neighbourfacecount = [0,0,0,0]
		for i,ek in enumerate(face.edge_keys):
			e_counter =0
			for face1 in obj.data.polygons:
				if ek in face1.edge_keys:
					e_counter += 1
			neighbourfacecount[i] = e_counter
		if len([x for x in neighbourfacecount if x > 3]) > 0:
			print(neighbourfacecount)
			face.select = True
				
		

#		for vert in verts_in_face:
#			print("vert", vert, " vert co", obj.data.vertices[vert].co)

def getEdges(obj):
	if obj.type != 'MESH':
		raise TypeError("Active object is not a Mesh")
	bpy.context.tool_settings.mesh_select_mode = (False, True, False) # face select
	for idx, edge in enumerate(obj.data.edges):
		

		print("edge index", edge.index) 
		if (idx % 2) == 0:
			print("selecting", idx)
			edge.select = True
			continue
		edge.select = False
	
#		for vert in verts_in_edge:
#			print("vert", vert, " vert co", obj.data.vertices[vert].co)

def getEdgesBMESH(obj):
	# force edge selection mode
	bpy.context.tool_settings.mesh_select_mode = (False, True, False) 
	
	# this will select every 'even' numbered edge from the list of edges.
	obj = bpy.context.active_object
	
	bm = bmesh.from_edit_mesh(obj.data)
	for edge in bm.edges:
		if edge.index % 2 == 0:
			edge.select = True
			continue
    
		edge.select = False	
	

if __name__ == "__main__":

	#getFaceEdges(bpy.context.object)
	
	bpy.ops.object.mode_set(mode='EDIT')
#	getEdges(bpy.context.active_object)
	
	getFaceEdgesBMESH(bpy.context.active_object)
#	bpy.ops.mesh.delete(type='ONLY_FACE') # delte only faces
	start = time()
#	for obj in bpy.data.objects:
#
#		getFaces(obj)
#		
#		getEdges(obj)
		
#		if obj.type == "MESH":
#			print(obj.name)
#			print("=====================Faces")
#			getFaces(obj)
#			print("=====================Edges")
#			getEdges(obj)
		
	print("this shit took %.2f seconds" % (time()-start))

