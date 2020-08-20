#!/usr/bin/env python3

"""
Simple exercise to construct a controller that controls the simulated Duckiebot using pose. 
"""

import time
import sys
import argparse
import math
from array import *
import numpy as np
import gym
from gym_duckietown.envs import DuckietownEnv

parser = argparse.ArgumentParser()
parser.add_argument('--env-name', default=None)
parser.add_argument('--map-name', default='udem1')
parser.add_argument('--no-pause', action='store_true', help="don't pause on failure")
args = parser.parse_args()

if args.env_name is None:
    env = DuckietownEnv(
        map_name = args.map_name,
        domain_rand = False,
        draw_bbox = False
    )
else:
    env = gym.make(args.env_name)

obs = env.reset()
env.render()

total_reward = 0

env.max_steps = math.inf
pi = math.pi

p = [[6.25, 1.75], [6.25, 4.25], [5.35, 4.25], [5.25, 5.25], [1.75, 5.25], [1.75, 1.75]]

tol = 0.1

io = 0

env.cur_pos = [1.5, 0, 1.9]
env.cur_angle = pi

def global_angle_arr(point, i):
	t = env.cur_angle
	x = env.cur_pos[0]
	y = env.cur_pos[2]
	rot = np.arctan2((y - point[i][1]), (point[i][0] - x))
	a =  rot - t
	a = (a/abs(a)) * (abs(a) % (2*pi))
	print(i, "t = ", t, "rot = ", rot, "a = ", a)
	if(a >= pi):
		a = -2*pi + a
		return a
	if(a < -pi):
		a = 2*pi - a
		return a
	return a
	
def global_angle(point):
	t = env.cur_angle
	x = env.cur_pos[0]
	y = env.cur_pos[2]
	rot = np.arctan2((y - point[1]), (point[0] - x))
	a =  rot - t
	a = (a/abs(a)) * (abs(a) % (2*pi))
	print(i, "t = ", t, "rot = ", rot, "a = ", a)
	if(a >= pi):
		a = -2*pi + a
		return a
	if(a < -pi):
		a = 2*pi - a
		return a
	return a	
	
	
def loca_angle(rad, dist):
	w = 10
	tile_size = 1
	return rad - (tile_size/20 - dist)*w

while True:

	lane_pose = env.get_lane_pos2(env.cur_pos, env.cur_angle)
	print("env.cur_pose =")
	print(env.cur_pos)
	print("\n")
	distance_to_road_center = lane_pose.dist
	angle_from_straight_in_rads = lane_pose.angle_rad
	print("dist = ", distance_to_road_center, " rad = ", angle_from_straight_in_rads)


	speed = 0.3
    

	if(abs(env.cur_pos[0] - p[io][0]) <= tol and abs(env.cur_pos[2] - p[io][1]) <= tol):
		io = io + 1
		if(io >= len(p)):
			io = 0

	obs, reward, done, info = env.step([speed, global_angle_arr(p, io)*2])
	#obs, reward, done, info = env.step([speed, loca_angle(angle_from_straight_in_rads, distance_to_road_center)])
	#obs, reward, done, info = env.step([speed, 0])
	#total_reward += reward
	print("info = ", info)
	#print('Steps = %s, Timestep Reward=%.3f, Total Reward=%.3f' % (env.step_count, reward, total_reward))

	env.render()

	if done:
		if reward < 0:
			print('*** CRASHED ***')
		print ('Final Reward = %.3f' % total_reward)
		break
