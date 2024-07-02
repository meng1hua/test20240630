import argparse
import carla
import cv2
import logging
import carla
from carla import ColorConverter as cc
import numpy as np
import time
import datetime
import weakref
import math
import os
import sys
import glob
import random
import pygame
import pandas as pd
from PIL import Image
from queue import Queue
from queue import Empty

IM_WIDTH = 800
IM_HEIGHT = 600
SHOW_PREVIEW = True

SHOW_CAM = SHOW_PREVIEW
im_width = IM_WIDTH
im_height = IM_HEIGHT

# 与服务器建立连接
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()
model_3 = blueprint_library.filter('model3')[0]

actor_list = []
transform = world.get_map().get_spawn_points()[100]
vehicle = world.spawn_actor(model_3, transform)
actor_list.append(vehicle)


def process_img(image):
    i = np.array(image.raw_data)
    i2 = i.reshape((im_height, im_width, 4))
    i3 = i2[:, :, : 3]
    if SHOW_CAM:
        cv2.imshow("", i3)
        cv2.waitKey(1)


cam = blueprint_library.find('sensor.camera.rgb')
cam.set_attribute('image_size_x', f'{im_width}')
cam.set_attribute('image_size_y', f'{im_height}')
cam.set_attribute('fov', f'110')

# 设定传感器的相对位置(x方向偏移2.5，z方向偏移0.7，y方向偏移)
# 需要调整传感器的角度可以在carla.Transform里添加carla.Rotation(roll,pitch,yew),分别代表x,y,z轴
# 不设置角度当前传感器与车头前向保持一直
transform = carla.Transform(carla.Location(x=2.5, z=0.7))
# 将传感器附在小车上
sensor = world.spawn_actor(cam, transform, attach_to=vehicle)
actor_list.append(sensor)
# 传感器开始监听
sensor.listen(lambda data: process_img(data))