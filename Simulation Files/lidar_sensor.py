from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla
from carla import ColorConverter as cc

import logging
import random
import re
import weakref
import numpy as np
import cv2
import time
import argparse
import time
from datetime import datetime
from matplotlib import cm
import open3d as o3d


VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
LABEL_COLORS = np.array([
    (255, 255, 255), # None
    (70, 70, 70),    # Building
    (100, 40, 40),   # Fences
    (55, 90, 80),    # Other
    (220, 20, 60),   # Pedestrian
    (153, 153, 153), # Pole
    (157, 234, 50),  # RoadLines
    (128, 64, 128),  # Road
    (244, 35, 232),  # Sidewalk
    (107, 142, 35),  # Vegetation
    (0, 0, 142),     # Vehicle
    (102, 102, 156), # Wall
    (220, 220, 0),   # TrafficSign
    (70, 130, 180),  # Sky
    (81, 0, 81),     # Ground
    (150, 100, 100), # Bridge
    (230, 150, 140), # RailTrack
    (180, 165, 180), # GuardRail
    (250, 170, 30),  # TrafficLight
    (110, 190, 160), # Static
    (170, 120, 50),  # Dynamic
    (45, 60, 150),   # Water
    (145, 170, 100), # Terrain
]) / 255.0 # normalize each channel [0-1] since is what Open3D uses

# ==============================================================================
# -- 2 Types of Lidars -------------------------------------------------------------
# ==============================================================================

def lidar_callback(point_cloud, point_list):
    """Prepares a point cloud with intensity
    colors ready to be consumed by Open3D"""
    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
    data = np.reshape(data, (int(data.shape[0] / 4), 4))

    # Isolate the intensity and compute a color for it
    intensity = data[:, -1]
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
    int_color = np.c_[
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]

    # Isolate the 3D data
    points = data[:, :-1]

    # We're negating the y to correclty visualize a world that matches
    # what we see in Unreal since Open3D uses a right-handed coordinate system
    points[:, :1] = -points[:, :1]

    # # An example of converting points from sensor to vehicle space if we had
    # # a carla.Transform variable named "tran":
    # points = np.append(points, np.ones((points.shape[0], 1)), axis=1)
    # points = np.dot(tran.get_matrix(), points.T).T
    # points = points[:, :-1]

    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)

def semantic_lidar_callback(point_cloud, point_list):
    """Prepares a point cloud with semantic segmentation
    colors ready to be consumed by Open3D"""
    data = np.frombuffer(point_cloud.raw_data, dtype=np.dtype([
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('CosAngle', np.float32), ('ObjIdx', np.uint32), ('ObjTag', np.uint32)]))

    # We're negating the y to correclty visualize a world that matches
    # what we see in Unreal since Open3D uses a right-handed coordinate system
    points = np.array([data['x'], -data['y'], data['z']]).T

    # # An example of adding some noise to our data if needed:
    # points += np.random.uniform(-0.05, 0.05, size=points.shape)

    # Colorize the pointcloud based on the CityScapes color palette
    labels = np.array(data['ObjTag'])
    int_color = LABEL_COLORS[labels]

    # # In case you want to make the color intensity depending
    # # of the incident ray angle, you can use:
    # int_color *= np.array(data['CosAngle'])[:, None]

    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)

# ==============================================================================
# -- Lidar_Sensor -------------------------------------------------------------
# ==============================================================================



def generate_lidar_bp(arg, world, blueprint_library, delta):
    """Generates a CARLA blueprint based on the script parameters"""
    if arg.semantic:
        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
    else:
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')

    lidar_bp.set_attribute('upper_fov', str(15.0))
    lidar_bp.set_attribute('lower_fov', str(25.0))
    lidar_bp.set_attribute('channels', str(64.0))
    lidar_bp.set_attribute('range', str(100))
    lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))
    lidar_bp.set_attribute('points_per_second', str(500000))
    return lidar_bp


# ==============================================================================
# -- Main_Function -------------------------------------------------------------
# ==============================================================================

def main(arg):
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(15.0)
    world = client.get_world()
    try:
        original_settings = world.get_settings()
        settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)

        delta = 0.05

        settings.fixed_delta_seconds = delta
        settings.synchronous_mode = True
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()
        blueprint_library = world.get_blueprint_library()

        bp = blueprint_library.filter('vehicle.tesla.model3')[0]
        print(bp)

        transform = carla.Transform()
        transform.location.x = 220
        transform.location.y = -1.85
        transform.location.z = 1.85
        transform.rotation.yaw = 180
        transform.rotation.pitch = 0
        transform.rotation.roll = 0
        vehicle = world.spawn_actor(bp, transform)
        spectator = world.get_spectator()
        sp_transform = carla.Transform(transform.location + carla.Location(z=30, x=-25),
        carla.Rotation(yaw=90, pitch=-90))
        spectator.set_transform(sp_transform)
        vehicle.set_autopilot(True)

        lidar_bp = generate_lidar_bp(arg, world, blueprint_library, delta)
        lidar_loc = carla.Location(0,0,0)
        lidar_rot = carla.Rotation(0,0,0)
        lidar_tra = carla.Transform(lidar_loc, lidar_rot)
        lidar = world.spawn_actor(lidar_bp, lidar_tra, attach_to=vehicle)
        point_list = o3d.geometry.PointCloud()
        if arg.semantic:
            lidar.listen(lambda data: semantic_lidar_callback(data, point_list))
        else:
            lidar.listen(lambda data: lidar_callback(data, point_list))

        vis = o3d.visualization.Visualizer()
        vis.create_window(
            window_name='Carla Lidar',
            width=960,
            height=540,
            left=480,
            top=270)
        vis.get_render_option().background_color = [0.05, 0.05, 0.05]
        vis.get_render_option().point_size = 1
        vis.get_render_option().show_coordinate_frame = True
        frame = 0
        dt0 = datetime.now()
        while True:
            if frame == 2:
                vis.add_geometry(point_list)
            vis.update_geometry(point_list)

            vis.poll_events()
            vis.update_renderer()
            # # This can fix Open3D jittering issues:
            time.sleep(0.005)
            world.tick()

            process_time = datetime.now() - dt0
            sys.stdout.write('\r' + 'FPS: ' + str(1.0 / process_time.total_seconds()))
            sys.stdout.flush()
            dt0 = datetime.now()
            frame += 1
    
    finally:
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)

        vehicle.destroy()
        lidar.destroy()
        vis.destroy_window()

if __name__ == "__main__":
    argparser = argparse.ArgumentParser(
        description=__doc__)

    argparser.add_argument(
        '--semantic',
        action='store_true',
        help='use the semantic lidar instead, which provides ground truth'
        ' information')
    args = argparser.parse_args()
    try:
        main(args)
    except KeyboardInterrupt:
        print(' - Exited by user.')
