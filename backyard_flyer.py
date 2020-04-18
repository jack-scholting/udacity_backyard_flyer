import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

# Indexes inside the target_position[] datastructure.
LON_IDX = 0
LAT_IDX = 1
ALT_IDX = 2

# Indexes for NEAH data structures
N_IDX = 0
E_IDX = 1
A_IDX = 2
H_IDX = 3

# The mission altitude (in meters)
MISSION_ALT = 3

# The legnth of one side of the mission square.
SQUARE_SIDE_LENGTH = 10

# Target Coordinates for a square
target_coords = [
    [SQUARE_SIDE_LENGTH, 0, MISSION_ALT, 0],
    [SQUARE_SIDE_LENGTH, SQUARE_SIDE_LENGTH, MISSION_ALT, 0],
    [0, SQUARE_SIDE_LENGTH, MISSION_ALT, 0],
    [0, 0, MISSION_ALT, 0]
]

LAST_WAYPOINT = len(target_coords)

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
        self.waypoint_num = 0

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data

        """
        # If TAKEOFF, and altitude reached, then WAYPOINT.
        if (self.flight_state == States.TAKEOFF):
            # coordinate conversion (local altitude is "down")
            altitude = -1.0 * self.local_position[ALT_IDX]

            # check if altitude is within 95% of target
            if altitude > 0.95 * self.target_position[ALT_IDX]:
                self.waypoint_transition()

        elif (self.flight_state == States.WAYPOINT):

            # Have we made it to the current target waypoint?
            waypoint_north = target_coords[self.waypoint_num][N_IDX]
            waypoint_east  = target_coords[self.waypoint_num][E_IDX]
            if ((abs(self.local_position[N_IDX] - waypoint_north) < 1) and
               (abs(self.local_position[E_IDX] - waypoint_east)) < 1):

                if (self.waypoint_num == LAST_WAYPOINT):
                    self.landing_transition()
                else:
                    self.waypoint_transition()


    def velocity_callback(self):
        """DONE
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        # Seems like this could work in the position interrupt too.
        if (self.flight_state == States.LANDING):
            # If we have reached the altitude of home (designed to handle non-0 home altitudes).
            # global_position - real altitude
            # global_home - saved GPS altitude of home
            # local_position - distance away from home altitude.
            if ((self.global_position[ALT_IDX] - self.global_home[ALT_IDX] < 0.1) and
                (abs(self.local_position[ALT_IDX]) < 0.01)):
                self.disarming_transition()


    def state_callback(self):
        """DONE
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        # Mostly taken from the UP/DOWN example.
        # This should be how we get out of that blocking .start() call.
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            if not self.armed:
                self.manual_transition()

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        pass

    def arming_transition(self):
        """DONE
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[LON_IDX] ,self.global_position[LAT_IDX], self.global_position[ALT_IDX])
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """DONE
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        self.target_position[ALT_IDX] = MISSION_ALT
        self.takeoff(MISSION_ALT)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        north = target_coords[self.waypoint_num][N_IDX]
        east  = target_coords[self.waypoint_num][E_IDX]
        alt   = target_coords[self.waypoint_num][A_IDX]
        hding = target_coords[self.waypoint_num][H_IDX]
        print("Commanding [{}, {}, {}, {}]".format(north, east, alt, hding))
        self.cmd_position(north, east, alt, hding)
        self.flight_state = States.WAYPOINT

        self.waypoint_num += 1

    def landing_transition(self):
        """DONE
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """DONE
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

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
        self.connection.start() # ARGH, this looks to be blocking, of course.

        # Why was this in solution but not in template?
        # print("Calling super().start()")
        # super().start()

        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()