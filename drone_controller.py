import time
import threading

from position import Position

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander



class DroneController:
    
    """
    This class controls the drone.

    Args:
        uri (str): URI radio channel.
        cache (str): Drone cache filename.
        origin_x (float): Drone origin position X axis.
        origin_y (float): Drone origin position Y axis.
        mix_x (float): Drone minimum position on X axis.
        mix_y (float): Drone minimum position on Y axis.
        max_x (float): Drone maximum position on X axis.
        max_y (float): Drone maximum position on Y axis.
        x_offset (float): Offset added to the position_to_visit.x in case of an existing offset between the ball position and drone position.
        y_offset (float): Offset added to the position_to_visit.y in case of an existing offset between the ball position and drone position.
        z_offset (float): Offset added to the position_to_visit.z in case of an existing offset between the ball position and drone position.
    """
    
    DEFAULT_HEIGHT = 1.0
    DEFAULT_VELOCITY = 0.8
    
    SLEEP_AFTER_VISIT = 0.1
    SLEEP_IN_LOOP = 1
    SLEEP_AFTER_TAKOFF = 5
    
    def __init__(self, uri='radio://0/100/2M/E7E7E7E701', cache='./cache',
                 origin_x=0.0, origin_y=0.0, min_x=0.0, min_y=-0.5, max_x=0.0,
                 max_y=0.0, x_offset=0.0, y_offset=0.0, z_offset=0.0):
        self.drone_number = uri[-1::]
        self.uri = uri
        self.cache = cache
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset
        self._position_to_visit = None
        self._last_position_visited = None
        self._land_now = False
        self._drone_started = False
        self.low_battery = False
    
    @property
    def position_to_visit(self):
        return self._position_to_visit
    
    @property
    def last_position_visited(self):
        return self._last_position_visited

    @property
    def land_now(self):
        return self._land_now
    
    @position_to_visit.setter
    def position_to_visit(self, position_to_visit):
        # Add the offsets for all axis
        if position_to_visit:
            position_to_visit.x = position_to_visit.x + self.x_offset
            position_to_visit.y = position_to_visit.y + self.y_offset
            position_to_visit.z = position_to_visit.z + self.z_offset
        self._position_to_visit = position_to_visit
            
    @last_position_visited.setter
    def last_position_visited(self, last_position_visited):
        self._last_position_visited = last_position_visited
        
    @land_now.setter
    def land_now(self, land_now):
        self._land_now = land_now
        
    @property
    def drone_started(self):
        return self._drone_started
    
    @drone_started.setter
    def drone_started(self, drone_started):
        self._drone_started = drone_started
    
    def go_to_position(self, controller):
        """
        Sends the drone to a specific position if he gets a new position.

        Args:
            controller (PositionHlCommander): Cflib PositionHlCommander.
        """
        
        position_x = round(self.position_to_visit.x, 2)
        position_y = round(self.position_to_visit.y, 2)
        position_z = round(self.position_to_visit.z, 2)
        
        # Don't go if the position is still the same
        if position_x == self.last_position_visited.x and position_y == self.last_position_visited.y:
            return
        
        print(f"[{str(self.drone_number)}] Going to position: ({str(position_x)}; {str(position_y)}; {str(position_z)})")

        controller.go_to(position_x, position_y, position_z)
        
        time.sleep(self.SLEEP_AFTER_VISIT)
        self.last_position_visited = self.position_to_visit
        self.position_to_visit = None
    
    def back_to_origin(self, controller):
        """
        Sends the drone back to the origin position.

        Args:
            controller (PositionHlCommander): Cflib PositionHlCommander.
        """
        print(f"[{str(self.drone_number)}] Going back to origin")
        controller.go_to(self.origin_x, self.origin_y, 0.5)
        time.sleep(self.SLEEP_AFTER_VISIT)
        
    def start(self, scf):
        
        """
        This is the drone's main loop where he waits for a new position to visit.

        Args:
            scf (SyncCrazyflie): Cflib SyncCrazyflie.
        """
        
        pc = PositionHlCommander(scf, controller=PositionHlCommander.CONTROLLER_PID,
                                default_velocity=self.DEFAULT_VELOCITY, default_height=self.DEFAULT_HEIGHT)
        
        print(f"[{str(self.drone_number)}] Taking off!")
        pc.take_off()
        
        time.sleep(self.SLEEP_AFTER_TAKOFF)
        
        self.last_position_visited = Position(0, 0, 0)
        
        while not self.land_now and threading.main_thread().is_alive():
            if self.low_battery:
                print(f"[{str(self.drone_number)}] Low battery!!")
                
            print(f"[{str(self.drone_number)}] Waiting for position")
            if self.position_to_visit:
                self.go_to_position(pc)
                
            # Sleep needed for CPU performance
            time.sleep(self.SLEEP_IN_LOOP) 
        
        self.back_to_origin(pc)
        time.sleep(self.SLEEP_AFTER_VISIT)
        
        pc.land()
            
    def main(self):
        cflib.crtp.init_drivers()
        
        # syncCrazyflie will create a synchronous Crazyflie instance with the specified link_uri.
        with SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache=self.cache)) as scf:     
            self.start(scf)
