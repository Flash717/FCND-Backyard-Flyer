import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


# note: use States to identify where in the process the drone currently is -> self.flight_state
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
        self.target_altitude = 3.0
        self.approximation = 0.95
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
        if self.flight_state == States.TAKEOFF:
            # if we are close to target_altitude then get the list of waypoints and start processing
            if self.local_position[2] > 0.95 * self.target_altitude:
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if abs(self.target_position[0] - self.local_position[0]) < self.approximation &
               abs(self.target_position[1] - self.local_position[1]) < self.approximation:
               if len(self.all_waypoints) > 0:
                   self.waypoint_transition()

               else:
                   # if we are close to the 'origin' 0 0 then land
                   if abs(self.local_position[0]) < self.approximation &
                        abs(self.local_position[1]) < self.approximation:
                      self.landing_transition()


    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.LANDING:
            self.disarming_transition()

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if self.in_mission == True:
            if self.flight_state == States.ARMING:
                if self.armed == True:
                    self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if (self.armed == False) & (self.guided == False):
                    self.manual_transition()
            elif self.flight_state == States.MANUAL:
                self.arming_transition()

    def calculate_box(self):
        # fly right up left, back home
        result = [[0.0, 10.0, 3.0],[10.0,10.0,3.0], [10.0, 0.0, 3.0], [0.0,0.0,3.0]]
        return result

    def arming_transition(self):
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0],
                                self.global_position[1],
                                self.global_position[2])
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        self.target_altitude = 3.0
        self.target_position[2] = self.target_altitude
        self.takeoff(self.target_position[2])
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        self.target_position = self.all_waypoints.pop(0)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
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
