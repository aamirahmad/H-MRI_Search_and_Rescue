# -*- coding: utf-8 -*-
"""
Created on Wed Dec 30 15:45:03 2015

@author: eruff

"""

import bpy 
import copy

from xml.etree.ElementTree import ElementTree, Element, SubElement, Comment
from xml.etree import ElementTree as ET
#from ElementTree_pretty import prettify

def indent(elem, level=0):
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i
            
def locrot2string(location,rot,name):
    # origin adjustments
    loc = copy.copy(location)
    if name.startswith('sidewall'):
        loc[2] -= 1
    if name.startswith('ramp'):
        loc[2] -= 0.0618276
    if name.startswith('uneven_surface'):
        loc[2] -= 0.1051185
    if name.startswith('barrel'):
        loc[2] -= 0.49
        
    string = ""
    for l in loc:
        string += str(l) + " "
    for r in rot:
        string += " " + str(r)
    return string

def addHeader(root):
    root.append(ET.fromstring('\
    <scene> \
          <ambient>0.5 0.5 0.5 1</ambient>\
          <background>0.5 0.5 0.5 1</background>\
          <shadows>false</shadows>\
        </scene>'))
    root.append(ET.fromstring('\
        <physics type="ode">\
          <gravity>0 0 -9.81</gravity>\
          <ode>\
            <solver>\
              <type>quick</type>\
              <iters>10</iters>\
              <sor>1.3</sor>\
            </solver>\
            <constraints>\
              <cfm>0.0</cfm>\
              <erp>0.2</erp>\
              <contact_max_correcting_vel>100.0</contact_max_correcting_vel>\
              <contact_surface_layer>0.001</contact_surface_layer>\
            </constraints>\
          </ode>\
          <real_time_update_rate>1000</real_time_update_rate>\
          <max_step_size>0.001</max_step_size>\
        </physics>'))
    
    root.append(ET.fromstring('\
        <light type="directional" name="directional_light_1">\
          <cast_shadows>false</cast_shadows>\
          <pose>0 0 30 0.1 0.1 0</pose>\
          <diffuse>1.0 1.0 1.0 1</diffuse>\
          <specular>.1 .1 .1 1</specular>\
          <attenuation>\
            <range>300</range>\
          </attenuation>\
          <direction>0.1 0.1 -1</direction>\
        </light>'))
    
def addGroundPlane(root):
    root.append(ET.fromstring('\
  <model name="ground_plane">\
      <pose>0 0 0  0 0 0</pose>\
      <static>true</static>\
      <link name="body">\
        <collision name="collision">\
          <geometry>\
	    <plane>\
              <normal>0 0 1</normal>\
            </plane>\
          </geometry>\
	<surface>\
            <friction>\
              <ode>\
                <mu>10.0</mu>\
                <mu2>10.0</mu2>\
                <fdir1>0 0 0</fdir1>\
                <slip1>0</slip1>\
                <slip2>0</slip2>\
              </ode>\
            </friction>\
            <bounce>\
              <restitution_coefficient>0</restitution_coefficient>\
              <threshold>1000000.0</threshold>\
            </bounce>\
            <contact>\
              <ode>\
                <soft_cfm>0</soft_cfm>\
                <soft_erp>0.2</soft_erp>\
                <kp>1e10</kp>\
                <kd>1</kd>\
                <max_vel>100.0</max_vel>\
                <min_depth>0.0001</min_depth>\
              </ode>\
            </contact>\
          </surface>\
        </collision>\
        <visual name="visual">\
          <geometry>\
            <mesh>\
              <uri>file://floor_plane.dae</uri>\
              <scale>1.5 1.5 1</scale>\
            </mesh>\
          </geometry>\
        </visual>\
      </link>\
    </model>'    ))
    
def getMesh(name):
    if name.startswith('sidewall'):
        return "file://sidewall.dae"
    if name.startswith('ramp'):
        return "file://ramp.dae"
    if name.startswith('uneven_surface'):
        return "file://uneven_surface.dae"
    if name.startswith('barrel'):
        return "file://barrel.dae"


def addGeometry(root,mesh_name,scale_str = "1 1 1"):
    geometry = SubElement(root,'geometry')
    mesh = SubElement(geometry,'mesh')
    uri = SubElement(mesh,'uri')
    uri.text = mesh_name
    scale = SubElement(mesh,'scale')
    scale.text = scale_str


def addCollision(root,mesh_name,scale):
    collision = SubElement(root,"collision",{"name":"collision"})
    addGeometry(collision,mesh_name,scale)
   
def addVisual(root,mesh_name,scale):
    visual = SubElement(root,"visual",{"name":"visual"})
    addGeometry(visual,mesh_name,scale)
    

def addModel(root,name,pose_str,mesh_name,scale_str = "1 1 1"):
    
    model = SubElement(root,"model",{"name":name})
    pose = SubElement(model,'pose')
    pose.text = pose_str
    static = SubElement(model,"static")
    static.text = "true"
    link = SubElement(model,"link",{"name":"body"})
    # add the collision element
    addCollision(link,mesh_name,scale_str)         
    # add the visual element
    addVisual(link,mesh_name,scale_str)
    
def main(world_name,saveToPath):
    
    root = Element('sdf')
    root.set('version', '1.4')

    comment = Comment('Generated by blend2world.py')
    root.append(comment)

    
    world = SubElement(root, 'world',{"name":world_name})
    
    addHeader(world)
    
    #add ground plane
    addGroundPlane(world)

#    inc = SubElement(world,'include')
#    uri = SubElement(inc,'uri')
#    uri.text = "model://sun"


    addModel(world,"rightsidewall","0 8.1 4  0 0 0","file://blackwall.dae")
    addModel(world,"backsidewall","-6 0.5 4  0 0 1.57","file://blackwall.dae","1.2833 1 1")
    addModel(world,"leftsidewall","0 -7.1 4  0 0 0","file://blackwall.dae")
    
    print("blend2world main function")
    for obs in bpy.data.objects:
        if obs.type == "MESH": 
            addModel(world,obs.name,locrot2string(obs.location,obs.rotation_euler,obs.name),getMesh(obs.name))
            

    indent(root)
    ET.dump(root)
    ElementTree(root).write(saveToPath)
