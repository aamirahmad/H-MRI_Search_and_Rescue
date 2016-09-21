# -*- coding: utf-8 -*-
"""
Created on Wed Dec 30 15:45:03 2015

@author: eruff

"""

import bpy 
import copy
import os
from numpy import linspace
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
            
def locrot2string(location,rot):
    loc = copy.copy(location)
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
              <iters>1000</iters>\
              <sor>1.3</sor>\
            </solver>\
            <constraints>\
              <cfm>0.0</cfm>\
              <erp>0.2</erp>\
              <contact_max_correcting_vel>100.0</contact_max_correcting_vel>\
              <contact_surface_layer>0.001</contact_surface_layer>\
            </constraints>\
          </ode>\
            <real_time_update_rate>120</real_time_update_rate> \
            <real_time_factor>1</real_time_factor> \
            <max_step_size>0.008333</max_step_size> \
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
    <!-- A ground plane --> \
        <include> \
            <uri>model://ground_plane</uri> \
        </include>'))
    """
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
              <uri>model://ground_plane</uri>\
              <scale>1.5 1.5 1</scale>\
            </mesh>\
          </geometry>\
        </visual>\
      </link>\
    </model>'    ))
    """
def getMesh(name):
    return "model://" + name + ".dae"


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

def addLight(root,name,pose_str):
    light = SubElement(root,"light",{"name":name,"type":"sun"})
    pose = SubElement(light,'pose')
    pose.text = pose_str
    light.append(ET.fromstring('<cast_shadows>1</cast_shadows>'))
    
    light.append(ET.fromstring('<diffuse>0.5 0.5 0.5 1</diffuse>'))
    light.append(ET.fromstring('<specular>0.1 0.1 0.1 1</specular>'))
    light.append(ET.fromstring('<attenuation>\
        <range>20</range>\
        <constant>0.5</constant>\
        <linear>0.01</linear>\
        <quadratic>0.00</quadratic>\
      </attenuation>'))

    light.append(ET.fromstring('<direction>0.1 0.1 -1.0</direction>' ))    

      

    

def includeModel(root,name,pose_str):
    
    include = SubElement(root,"include")
    inc_name = SubElement(include,"name")
    inc_name.text = name
    pose = SubElement(include,'pose')
    pose.text = pose_str
    uri = SubElement(include,"uri")
    uri.text = "model://" + name.split(".")[0]
    
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

#
#    addModel(world,"rightsidewall","0 8.1 4  0 0 0","file://blackwall.dae")
#    addModel(world,"backsidewall","-6 0.5 4  0 0 1.57","file://blackwall.dae","1.2833 1 1")
#    addModel(world,"leftsidewall","0 -7.1 4  0 0 0","file://blackwall.dae")
#    
    
    for i in linspace(-10,10,3):
        for j in linspace(-12,12,3):
            light_name ="directional_light."+ str(i) + "_" + str(j)
            addLight(world,light_name,locrot2string([i,j,8],[0,0,0]))
            
        
    print("blend2world main function")
    models = os.listdir(bpy.path.abspath('//')+"models/")
    # get the current scene
    scn = bpy.context.scene
    for obs in scn.objects:
        if obs.type == "MESH": 
            model_name = obs.name.split(".")[0]
            if model_name in models:
                includeModel(world,obs.name,locrot2string(obs.location,obs.rotation_euler))
            else:
                print("no such model found:",model_name)
                
            

    indent(root)
    #ET.dump(root)

    ElementTree(root).write(saveToPath + world_name + ".world")



if __name__ == '__main__':
    # get the current scene
    filename = bpy.path.basename(bpy.context.blend_data.filepath).split(".")[0]
    filename = bpy.context.scene.name
    main(filename,bpy.path.abspath('//') + "worlds/")