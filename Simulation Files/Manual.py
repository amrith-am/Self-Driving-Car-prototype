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

# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================
#          F1      restart
#          F5      toggle _autopilot_enabled
#          F9      recording
#          F11     next_weather    reversed
#          F12     next_weather
#          0-9     set_sensors
#          w           throtle
#    a         d       steer 
#       z              reverse control
#         
#          s       brake
#          h       hand-brake
#          tab     toggle camera
class KeyboardControl(object):
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        self._control = carla.VehicleControl()
        self._steer_cache = 0.0
        world.vehicle.set_autopilot(self._autopilot_enabled)
        #world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, world, keys):
        if(True):
                if (keys == 65470):   # F1, event.key == K_BACKSPACE:
                    world.restart()
                elif (keys == 9):   # TAB, event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif (keys == 65480):   # F11, event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif (keys == 65481):   # F12, event.key == K_c:
                    world.next_weather()
                elif (keys == ord('0')):   # event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif (keys == ord('1')):   # event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(1 - 1)
                elif (keys == ord('2')):   # event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(2 - 1)
                elif (keys == ord('3')):   # event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(3 - 1)
                elif (keys == ord('4')):   # event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(4 - 1)
                elif (keys == ord('5')):   # event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(5 - 1)
                elif (keys == ord('6')):   # event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(6 - 1)
                elif (keys == ord('7')):   # event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(7 - 1)
                elif (keys == ord('8')):   # event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(8 - 1)
                elif (keys == ord('9')):   # event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(9 - 1)
                elif (keys == 65478):   # F9, event.key == K_r:
                    world.camera_manager.toggle_recording()
                elif (keys == ord('z')):   # Z, event.key == K_q:
                    self._control.reverse = not self._control.reverse
                elif (keys == 65474):   # F5, event.key == K_p:
                    self._autopilot_enabled = not self._autopilot_enabled
                    world.vehicle.set_autopilot(self._autopilot_enabled)
                    world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
        if not self._autopilot_enabled:
            self._parse_keys( key, int(round(time.time() * 1000)) )  # pygame.key.get_pressed(), clock.get_time())
            world.vehicle.apply_control(self._control)

    def _parse_keys(self, keys, milliseconds):
        self._control.throttle = 1.0 if keys == ord('w') else 0.0
        steer_increment = 5e-4 * milliseconds
        if (keys == ord('a')):   # keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif (keys == ord('d')):   # keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys == ord('s') else 0.0
        self._control.hand_brake = True if keys == ord('h') else False  # keys[K_SPACE]

# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        self._text = 'Autopilot off'

    def tick(self, world, clock):
        pass

    def notification(self, text, seconds=2.0):
        self._text = text

    def error(self, text):
        self._text = text

    def render(self, image):
        if(image is not None):
            cv2.putText(image, self._text, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
            cv2.imshow('MANUAL', image)
            
        key = cv2.waitKeyEx(30)
        if(key == 27):
            cv2.destroyAllWindows()
        return(key)


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


START_POSITION = carla.Transform(carla.Location(x=180.0, y=199.0, z=40.0))


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


class World(object):
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.hud = hud
        blueprint = self._get_random_blueprint()
        self.vehicle = self.world.spawn_actor(blueprint, START_POSITION)
        self.camera_manager = CameraManager(self.vehicle, self.hud, args)
        self.camera_manager.set_sensor(0, notify=False)
        self.controller = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self.args = args

    def restart(self):
        cam_index = self.camera_manager._index
        cam_pos_index = self.camera_manager._transform_index
        start_pose = self.vehicle.get_transform()
        start_pose.location.z += 2.0
        start_pose.rotation.roll = 0.0
        start_pose.rotation.pitch = 0.0
        blueprint = self._get_random_blueprint()
        self.destroy()
        self.vehicle = self.world.spawn_actor(blueprint, start_pose)
        self.camera_manager = CameraManager(self.vehicle, self.hud, self.args)
        self.camera_manager._transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = ' '.join(self.vehicle.type_id.replace('_', '.').title().split('.')[1:])
        if(self.hud is not None):
            self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        if(self.hud is not None):
            self.hud.notification('Weather: %s' % preset[1])
        self.vehicle.get_world().set_weather(preset[0])

    def tick(self, clock):
        if(self.hud is not None):
            self.hud.tick(self, clock)

    def render(self, display):
        ret = 0
        image = self.camera_manager.render(display)
        if(self.hud is not None):
            ret = self.hud.render(image)
        return ret

    def destroy(self):
        for actor in [self.camera_manager.sensor, self.collision_sensor.sensor, self.vehicle]:
            if actor is not None:
                actor.destroy()

    def _get_random_blueprint(self):
        bp = random.choice(self.world.get_blueprint_library().filter('vehicle').filter('vehicle.tesla.model3'))
        # jeff, use jeep only
        print("bp", bp.tags)
        return bp

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud, args):
        self.sensor = None
        self._surface = None
        self._parent = parent_actor
        self._hud = hud
        self._recording = False
        self._camera_transforms = [
            carla.Transform(carla.Location(x=1.6, z=1.7)),
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))]
        self._transform_index = 1
        self._sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)']]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        
        for item in self._sensors:
            bp = bp_library.find(item[0])
            if(hud is not None):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
            else:  # 1280x720
                bp.set_attribute('image_size_x', str(args.width) )
                bp.set_attribute('image_size_y', str(args.height) )
            item.append(bp)
        self._index = None
        #self._server_clock = pygame.time.Clock()
        self._image_raw = None
        #print('_server_clock', self._server_clock)

    def toggle_camera(self):
        self._transform_index = (self._transform_index + 1) % len(self._camera_transforms)
        self.sensor.set_transform(self._camera_transforms[self._transform_index])

    def set_sensor(self, index, notify=True):
        index = index % len(self._sensors)
        needs_respawn = True if self._index is None \
            else self._sensors[index][0] != self._sensors[self._index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self._surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self._sensors[index][-1],
                self._camera_transforms[self._transform_index],
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify and (self._hud is not None):
            self._hud.notification(self._sensors[index][2])
        self._index = index

    def next_sensor(self):
        self.set_sensor(self._index + 1)

    def toggle_recording(self):
        self._recording = not self._recording
        if(self._hud is not None):
            self._hud.notification('Recording %s' % ('On' if self._recording else 'Off'))

    def render(self, display):
        #if self._surface is not None:
        #    display.blit(self._surface, (0, 0))
        return (self._image_raw)

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        #self._server_clock.tick()
        #if(self._hud is not None):
        #    self._hud.server_fps = self._server_clock.get_fps()
        image.convert(self._sensors[self._index][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        # 500. save raw image
        self._image_raw = array.copy()
        #jeff
        return
        '''
        array = array[:, :, ::-1]
        self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self._recording:
            image.save_to_disk('_out/%08d' % image.frame_number)
        '''

# ==============================================================================
# -- mc_args -------------------------------------------------------------
# ==============================================================================

class mc_args(object):
    def __init__(self):
        self.debug = True
        self.host = '127.0.0.1'
        self.port = 2000
        self.autopilot = False  # True
        self.width =  1840  # 1280
        self.height = 960   # 720
args = mc_args()


# 100. initalizing client only once

client = carla.Client(args.host, args.port)
client.set_timeout(20.0)
hud = HUD(args.width, args.height)

# 200.
world = None
try:
    world = World(client.get_world(), hud, args)   
    controller = KeyboardControl(world, args.autopilot)
    while ( True ):
        key = world.render(None)
        controller.parse_events(world, key)
        if (key == 27):
            break
    
finally:  
    if world is not None:
        world.destroy()
