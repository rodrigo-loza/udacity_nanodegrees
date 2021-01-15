import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """        
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            # Load waypoints
            self.calculate_box()                        
            self.takeoff_transition()
        elif self.flight_state == States.TAKEOFF:
            # arbitrary criteria of 0.3m altitude for assuming flight altitude has been reached before
            # transitioning to waypoint navigation
            if abs(-self.local_position[2] - self.target_position[2]) < 0.3:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            # calculate the xy distance from target position
            dist = np.linalg.norm(self.local_position[:-1] - self.target_position[:-1])
            # arbitrary criteria of 0.3m for assuming current target position has been reached
            if dist < 0.3:
                self.waypoint_transition()
        elif self.flight_state == States.LANDING:
            # arbitrary criteria of 0.1m altitude for assuming ground level has been safely reached
            # before disarming
            if self.local_position[2] > -0.1:
                self.disarming_transition()


    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        pass

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        pass

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        box_waypoints = [np.array([10.0, 0.0]), np.array([10.0, 10.0]), np.array([0, 10.0]), np.array([0, 0])]
        for waypoint in box_waypoints:
            self.all_waypoints.append(waypoint)

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drones
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        # Function #1
        self.take_control()
        time.sleep(2)
        # Function #2
        self.arm()
        # Function #3
        self.set_home_as_current_position()
        # Function #4
        self.flight_state = States.ARMING               

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print(f'Takeoff location: {self.local_position}')
        print("takeoff transition")
        # Function #1
        self.target_position[2] = 3
        # Function #2
        self.takeoff(self.target_position[2])
        # Function #3
        self.flight_state = States.TAKEOFF        

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """        
        # check if thers is any waypoint not visited
        if len(self.all_waypoints) > 0:
            print("waypoint transition")            
            # update target position with next waypoint from calculate_box
            self.target_position[:2] = self.all_waypoints.pop(0)
            print(f'target position {self.target_position}')
            # Function #1
            self.cmd_position(*self.target_position, 0)
            # Function #2
            self.flight_state = States.WAYPOINT            
            
        else:
            # all waypoints have been visited
            self.landing_transition()        

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        # Function #1
        self.land()
        # Function #2
        self.flight_state = States.LANDING        

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print(f'Landing location: {self.local_position}')
        print("disarm transition")
        # Function #1
        self.disarm()
        # Function #2
        self.flight_state = States.DISARMING
        time.sleep(2)        
        self.manual_transition()

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
