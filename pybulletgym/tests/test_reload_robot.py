

import xml.etree.ElementTree as ET
import os 
import numpy as np 
import time 

printf = lambda name, prev, new : print(f"{name:10s} : {prev} -> {new}")

def modify_xml(dir_path, xml, save_name, seed=0):
    with open(os.path.join(dir_path,xml)) as f :
        tree = ET.parse(f)
        root = tree.getroot()
    worldbody = root.find("worldbody").find("body")
    bodies = worldbody.findall("body")
    for b in bodies:
        for bb in b.findall("body"):
            for bbb in bb.findall("body"):
                for geo in bbb.findall("geom"):
                    attrib="fromto"
                    geo_size = geo.get(attrib)
                    geo_size = list(map(float, geo_size.split()))
                    geo_size = list(map(lambda x:str(np.round(x*(np.random.random()*2),2)), geo_size))
                    geo_size = " ".join(geo_size)
                    geo.set('fromto', geo_size)
                    printf(geo.get(attrib), geo_size, geo.get(attrib))
    if os.path.exists(os.path.join(dir_path, save_name)):
        os.remove(os.path.join(dir_path, save_name))
    with open(os.path.join(dir_path, save_name), "wb") as file:
        tree.write(file, encoding='utf-8', xml_declaration=True)
        
        
import gym
import pybulletgym 
import pybullet as p
# modify_xml("pybulletgym/envs/assets/mjcf", "ant.xml", "ant_v1.xml")
env = gym.make("AntPyBulletEnv-v0", xml="ant.xml")
env.render()
env.reset()
p.resetDebugVisualizerCamera(5.5, 1.0, -35.0, (0.0,0.0,1.0))

from pybulletgym.envs.roboschool.robots.locomotors import Ant


done = False 
timestep =0
episode = 0
while not done:
    state, reward, done, info = env.step(env.action_space.sample())
    time.sleep(0.02)
    timestep += 1
    if timestep >100:
        done = True
    if done:
        timestep = 0
        done = False 
        name = str(f"ant_overwritten.xml")
        modify_xml("pybulletgym/envs/assets/mjcf", "ant.xml", name)
        robot = Ant(name)
        env.reload_robot(robot)
        env.reset()
        p.resetDebugVisualizerCamera(5.5, 1.0, -35.0, (0.0,0.0,1.0))
        episode +=1 