import  carla
import  random
from carla import Map
from carla import Client

from carla import  *
from libcarla import  *
client:carla.Client = carla.Client("localhost",2000)
world:carla.World = client.get_world()

# client.load_world("Town06")
spectator:carla.Actor = world.get_spectator()

transform:carla.Transform =spectator.get_transform()
location = transform.location
rotation = transform.rotation

spectator.set_transform(carla.Transform())

vehicle:BlueprintLibrary = world.get_blueprint_library().filter("*vehicle*")

spwan_points:Map = world.get_map()
spwan_point = spwan_points.get_spawn_points()
# for i in range(20):
#     world.try_spawn_actor(random.choice(vehicle),random.choice(spwan_point))
ego_vehicle = world.spawn_actor(random.choice(vehicle),random.choice(spwan_point))
camera_init_trans:Transform = carla.Transform(carla.Location(x=1.0,z=1.5))
camera_bp:BlueprintLibrary = world.get_blueprint_library().find("sensor.camera.rgb")

camera:Actor = world.spawn_actor(camera_bp,camera_init_trans,attach_to = ego_vehicle)
camera.listen(lambda  image:image.save_to_disk("D:\\opt\\carla\\PythonAPI\\examples\\outout\\%06d.png"%image.frame))
