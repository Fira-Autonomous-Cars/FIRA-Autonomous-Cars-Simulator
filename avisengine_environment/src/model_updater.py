#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import os 


dir_path = os.path.dirname(os.path.realpath(__file__))
parent_dir = "/src"
parent_dir_path = dir_path[0:len(dir_path) - len(parent_dir)]


tree = ET.parse(parent_dir_path + "/worlds/world_race.world")
root = tree.getroot()

mesh_path = parent_dir_path + "/meshes/mesh_road.dae"


for item in root.findall('./world/model/link/visual/geometry/mesh/uri'):
    item.text = mesh_path 
    print("Successfully updated path : {}".format(item.text))


for item in root.findall('./world/model/link/collision/geometry/mesh/uri'):
    item.text = mesh_path
    print("Successfully updated path : {}".format(item.text))

tree.write(parent_dir_path + "/worlds/world_race.world")


