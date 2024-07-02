import glob
import os
import sys
import time
import argparse
import carla
from pygame.locals import K_ESCAPE
from pygame.locals import K_q
import random
import numpy as np

from queue import Queue, Empty
import random
import pygame

random.seed(10)  # 决定车辆生成新位置


# 显示控制界面
class DisplayManager:
    def __init__(self, grid_size, window_size):
        # pygame进行初始化
        pygame.init()
        # 字体进行初始化
        pygame.font.init()
        # 设置窗口属性
        self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        # 窗口布局
        self.grid_size = grid_size
        # 窗口的大小
        self.window_size = window_size
        # 传感器列表
        self.sensor_list = []

    # 窗口的大小
    def get_window_size(self):
        return [int(self.window_size[0]), int(self.window_size[1])]

    # 每个显示窗口的大小
    def get_display_size(self):
        return [int(self.window_size[0] / self.grid_size[1]), int(self.window_size[1] / self.grid_size[0])]

    # 窗口显示的动态偏移量
    def get_display_offset(self, gridPos):
        dis_size = self.get_display_size()
        return [int(gridPos[1] * dis_size[0]), int(gridPos[0] * dis_size[1])]

    # 添加数据到传感器列表中
    def add_sensor(self, sensor):
        self.sensor_list.append(sensor)

    # 获得传感器列表
    def get_sensor_list(self):
        return self.sensor_list

    # 进行数据的刷新
    def render(self):
        if not self.render_enabled():
            return

        for s in self.sensor_list:
            s.render()

        pygame.display.flip()

    # 清理传感器对象
    def destroy(self):
        for s in self.sensor_list:
            s.destroy()

    # 设置是否可以进行显示
    def render_enabled(self):
        return self.display != None


# 创建一个定时器
class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    # 返回自epoch直到现在时刻的秒数
    def time(self):
        return self.timer()


# 传感器数据状态管理
class SensorManager:
    def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos):
        # pygame 进行显示的渲染器
        self.surface = None
        # 世界对象
        self.world: carla.World = world
        # 显示管理对象
        self.display_man: DisplayManager = display_man
        # 显示的位置
        self.display_pos = display_pos
        # 初始化传感器
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        # 像机参数设置
        self.sensor_options = sensor_options
        # 自定义定时器
        self.timer = CustomTimer()
        # 处理耗时
        self.time_processing = 0.0
        # 处理次数
        self.tics_processing = 0

        self.display_man.add_sensor(self)

    # 传感器对象初始化
    # sensor_type 传感器的类型
    # transform 传感器的位置
    # attached 传感器绑定对象
    # sensor_options 选项参数
    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        # 将RGB数据输出到pygame中
        if sensor_type == 'RGBCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)

            return camera
        # 设置雷达传感器
        elif sensor_type == 'LiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('range', '100')
            lidar_bp.set_attribute('dropoff_general_rate',
                                   lidar_bp.get_attribute('dropoff_general_rate').recommended_values[0])
            lidar_bp.set_attribute('dropoff_intensity_limit',
                                   lidar_bp.get_attribute('dropoff_intensity_limit').recommended_values[0])
            lidar_bp.set_attribute('dropoff_zero_intensity',
                                   lidar_bp.get_attribute('dropoff_zero_intensity').recommended_values[0])

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_lidar_image)

            return lidar
        # 设置分割雷达传感器
        elif sensor_type == 'SemanticLiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
            lidar_bp.set_attribute('range', '100')

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_segmanticlidar_image)

            return lidar
        # 极端雷达传感器
        elif sensor_type == "Radar":
            radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
            for key in sensor_options:
                radar_bp.set_attribute(key, sensor_options[key])

            radar = self.world.spawn_actor(radar_bp, transform, attach_to=attached)
            radar.listen(self.save_radar_image)

            return radar

        else:
            return None

    def get_sensor(self):
        return self.sensor

    # 保存RGB图像
    def save_rgb_image(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        # 进行对应数据的显示
        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        # 解析消耗时间
        t_end = self.timer.time()
        self.time_processing += (t_end - t_start)
        self.tics_processing += 1

    # 保存对应的雷达数据
    def save_lidar_image(self, image):
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        # 获取雷达的范围信息
        lidar_range = 2.0 * float(self.sensor_options['range'])
        # 将其转为numpy数据
        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        # 只是用前X，Y数据
        lidar_data = np.array(points[:, :2])
        # 缩放雷达的数据使用窗口的大小
        lidar_data *= min(disp_size) / lidar_range
        # 进行数据的偏移
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        # 对所有的数据取绝对值
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        # 转为32位整数
        lidar_data = lidar_data.astype(np.int32)
        # 所有的数据重设置n,2类型
        lidar_data = np.reshape(lidar_data, (-1, 2))
        # 制作同样大小的图像，通道数为3
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        # 初始化为0
        lidar_img = np.zeros(lidar_img_size, dtype=np.uint8)
        # 填充对应的数据
        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end - t_start)
        self.tics_processing += 1

    # 语义激光雷达
    def save_segmanticlidar_image(self, image):
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0 * float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 6), 6))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros(lidar_img_size, dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end - t_start)
        self.tics_processing += 1

    def save_radar_image(self, radar_data):
        t_start = self.timer.time()
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (len(radar_data), 4))

        t_end = self.timer.time()
        self.time_processing += (t_end - t_start)
        self.tics_processing += 1

    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)

    def destroy(self):
        self.sensor.destroy()


def main(args):
    args.width, args.height = [int(x) for x in args.image_size.split('x')]
    display_manager = None
    vehicle = None
    vehicle_list = []
    timer = CustomTimer()

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)
        world: carla.World = client.get_world()
        orignal_settings: carla.WorldSettings = world.get_settings()
        spectator:carla.Actor = world.get_spectator()
        # 同步设置
        if args.sync:
            traffic_manger: carla.TrafficManager = client.get_trafficmanager(args.tm_port)
            setting: carla.WorldSettings = world.get_settings()
            traffic_manger.set_synchronous_mode(True)
            traffic_manger.set_global_distance_to_leading_vehicle(2.5)
            setting.synchronous_mode = True
            setting.fixed_delta_seconds = 0.05
            world.apply_settings(setting)
        # 生成车辆
        blueprints: carla.BlueprintLibrary = world.get_blueprint_library().filter("*vehicle*")
        # 获得地图
        this_map: carla.Map = world.get_map()
        # 获得地图的随机生成点
        spawn_points = this_map.get_spawn_points()
        # 部署20个地图到场景中
        for i in range(20):
            actor:carla.Vehicle = world.try_spawn_actor(random.choice(blueprints), random.choice(spawn_points))
            # actor.set_autopilot(True)
            vehicle_list.append(actor)


        # 自定义车辆到地图中
        this_blueprint_library: carla.BlueprintLibrary = world.get_blueprint_library()
        ego_bp: carla.BlueprintLibrary = this_blueprint_library.find("vehicle.tesla.cybertruck")
        ego_bp.set_attribute('role_name', 'hero')
        vehicle: carla.Vehicle = world.spawn_actor(ego_bp, random.choice(spawn_points))

        # vehicle.set_autopilot(True)
        vehicle_list.append(vehicle)

        # 设置车辆自动控制状态
        for vehicle in world.get_actors().filter("*vehicle*"):
            vehicle:carla.Vehicle
            vehicle.set_
            vehicle.set_autopilot(True)

        display_manager = DisplayManager(grid_size=[2, 3], window_size=[args.width, args.height])
        # 绑定传感器到车上
        SensorManager(world, display_manager, 'RGBCamera',
                      carla.Transform(carla.Location(x=0, z=3.0), carla.Rotation(yaw=-90)),
                      vehicle, {}, display_pos=[0, 0])
        SensorManager(world, display_manager, 'RGBCamera',
                      carla.Transform(carla.Location(x=0, z=3.0), carla.Rotation(yaw=+00)),
                      vehicle, {}, display_pos=[0, 1])
        SensorManager(world, display_manager, 'RGBCamera',
                      carla.Transform(carla.Location(x=0, z=3.0), carla.Rotation(yaw=+90)),
                      vehicle, {}, display_pos=[0, 2])
        SensorManager(world, display_manager, 'RGBCamera',
                      carla.Transform(carla.Location(x=0, z=3.0), carla.Rotation(yaw=180)),
                      vehicle, {}, display_pos=[1, 1])

        SensorManager(world, display_manager, 'LiDAR', carla.Transform(carla.Location(x=0, z=2.4)),
                      vehicle,
                      {'channels': '64', 'range': '100', 'points_per_second': '500000', 'rotation_frequency': '20'},
                      display_pos=[1, 0])
        SensorManager(world, display_manager, 'SemanticLiDAR', carla.Transform(carla.Location(x=0, z=2.4)),
                      vehicle,
                      {'channels': '64', 'range': '100', 'points_per_second': '500000', 'rotation_frequency': '20'},
                      display_pos=[1, 2])
        # -------------------------- 传感器配置完成，进行实验 --------------------------#
        call_exit = False
        time_init_sim = timer.time()
        while True:
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()
            display_manager.render()
            transform:carla.Transform = vehicle.get_transform();
            spectator.set_transform(transform)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    call_exit = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == K_ESCAPE or event.key == K_q:
                        call_exit = True
                        break
                if call_exit:
                    break

    finally:

        if display_manager:
            display_manager.destroy()
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])
        world.apply_settings(orignal_settings)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', metavar='H', default='localhost', help='IP of the host server (default: 127.0.0.1)')
    parser.add_argument('--port', '-p', default=2000, type=int, help='TCP port to listen to (default: 2000)')
    parser.add_argument('--tm_port', default=8000, type=int, help='Traffic Manager Port (default: 8000)')
    parser.add_argument('--ego-spawn', type=list, default=None, help='[x,y] in world coordinate')
    parser.add_argument('--top-view', default=True, help='Setting spectator to top view on ego car')
    parser.add_argument('--map', default='Town04', help='Town Map')
    parser.add_argument('--sync', default=True, help='Synchronous mode execution')
    parser.add_argument('--sensor-h', default=2.4, help='Sensor Height')
    parser.add_argument('--save-path', default='D:\\code\\out', help='Synchronous mode execution')
    parser.add_argument("--image_size", default="1152x648", help='image_width x image_heght')
    
    args = parser.parse_args()
    main(args)
