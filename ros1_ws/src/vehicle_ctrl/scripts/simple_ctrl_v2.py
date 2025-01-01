#!/usr/bin/env python3

import sys
import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
sys.path.append('/opt/carla-simulator/PythonAPI/carla')
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO 
import carla
from carla import Client, Color
from carla import Location, Transform
# import agents.navigation.controller as ctrl
from lat_long_ctrl import VehiclePIDController
import math
from carla_msgs.msg import CarlaEgoVehicleControl
# from tf_transformations import euler_from_quaternion
import tf2_ros
from tf2_ros import TransformListener, Buffer

RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
MAGENTA = "\033[35m"
CYAN = "\033[36m"
WHITE = "\033[37m"
RESET = "\033[0m"


class CarlaVehicleControl(object):
    def __init__(self):
        # Placeholders for start and end poses
        self.start_pose = None
        self.end_pose = None
        
        self.waypoints_list = []  # Initialize as an empty list
        rospy.loginfo(f"DEBUG: waypoints_list is now {self.waypoints_list}")

        self.odom = None
        
        # TF2 buffer and listener for transform lookup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Subscribers
        self.initialpose_sub = rospy.Subscriber(
            '/initialpose',
            PoseWithCovarianceStamped,
            self.initialpose_callback
        )
        self.goal_pose_sub = rospy.Subscriber(
            '/move_base_simple/goal',
            PoseStamped,
            self.goal_pose_callback
        )
        self.waypt_sub = rospy.Subscriber(
            '/carla/ego_vehicle/waypoints',
            Path,
            self.waypoints_callback
        )
        # # Subscriber to the /carla/ego_vehicle/odometry topic
        self.odom_sub = rospy.Subscriber(
            '/carla/ego_vehicle/odometry',
            Odometry,
            self.odometry_callback
        )
        # Subscriber to the /carla/ego_vehicle/odometry topic
        # self.odom_sub = rospy.Subscriber(
        #     '/odometry_from_dynamic_tf', # /aft_mapped_to_init_high_frec
        #      Odometry,
        #     self.odometry_callback
        # )

        self.vehicle_control_publisher = rospy.Publisher(
            '/carla/ego_vehicle/vehicle_control_cmd',
            CarlaEgoVehicleControl,
            queue_size=10
        )

        # Initialize Carla client and map
        self.client = Client('localhost', 2000)
        self.client.set_timeout(10.0)

        # Get the current world
        self.world = self.client.get_world()

        # Check if Town01 is already loaded
        if 'Town01' not in self.world.get_map().name:
            print("Town01 is not loaded. Loading Town01...")
            self.world = self.client.load_world('Town01')
            print("Done!")
        else:
            print("Town01 is already loaded.")

        self.map = self.world.get_map()
        # Initialize GlobalRoutePlanner
        # self.route_planner = GlobalRoutePlanner(self.map, 2.0)
        self.dao = GlobalRoutePlannerDAO(self.map, 2.0)
        self.route_planner = GlobalRoutePlanner(self.dao)

        # Get all actors (vehicles, pedestrians, etc.) in the world
        self.actors = self.world.get_actors()

        # Filter to get only the vehicles get the 0-th veh as there is only one veh
        self.vehicle = self.actors.filter('vehicle.*')[0]

        

        # TF2 listener and buffer
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer)
        # self.vehicle_loc = None

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw).
        """
        x, y, z, w = quaternion
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw
    
    def odometry_callback(self, msg):
        rospy.loginfo(
            f"Received odometry data: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}"
        )
        print(f"\n{GREEN} $$$$$$$$$$ Received odometry data: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}{RESET}\n")

        # Extract position and orientation from Odometry message
        x = msg.pose.pose.position.x
        y = -msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        print(" ^^^^ ODOM XYZ: ", x, y, z)

        orientation_q = msg.pose.pose.orientation
        roll, pitch, yaw = self.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Create a carla.Location object
        location = carla.Location(x=x, y=y, z=z)

        # Create a carla.Rotation object
        rotation = carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)

        # Create a carla.Transform object
        transform = carla.Transform(location, rotation)

        self.odom = transform

    def waypoints_callback(self, msg):
        # Iterate through all the waypoints in the Path message
        for pose in msg.poses:
            # Extract the position from the pose
            x = pose.pose.position.x
            y = -pose.pose.position.y
            z = pose.pose.position.z

            # Extract the orientation (quaternion) from the pose
            orientation_q = pose.pose.orientation
            roll, pitch, yaw = self.euler_from_quaternion(
                [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

            # Create a carla.Location object
            location = carla.Location(x=x, y=y, z=z)

            # Create a carla.Rotation object
            rotation = carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)

            # Create a carla.Transform object
            transform = carla.Transform(location, rotation)

            # Store the Waypoint in the global list
            self.waypoints_list.append(transform)

            rospy.loginfo(
                f"Stored {len(self.waypoints_list)} waypoints as carla.libcarla.Waypoint objects."
            )

    def create_ctrl_msg(self, throttle, steer, brake):
        control_msg = CarlaEgoVehicleControl()
        control_msg.throttle = throttle
        control_msg.steer = steer
        control_msg.brake = brake
        return control_msg

    def initialpose_callback(self, msg):
        rospy.loginfo("Received initialpose")
        self.start_pose = msg.pose.pose

    def goal_pose_callback(self, msg):
        rospy.loginfo("Received goal_pose")
        self.end_pose = msg.pose
        # Clear the waypoints list for the new goal
        self.waypoints_list.clear()

    def get_transform(self, vehicle_location, angle, d=6.4):
        a = math.radians(angle)
        location = carla.Location(
            d * math.cos(a), d * math.sin(a), 2.0) + vehicle_location
        return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-15))

    def setup_PID(self, vehicle):
        """
        This function creates a PID controller for the vehicle passed to it 
        """
        args_lateral_dict = {
            'K_P': 0.5,  # Reduced proportional gain for smoother steering
            'K_D': 0.1,  # Small derivative gain to dampen oscillations
            'K_I': 0.01, # Small integral gain to correct for long-term drift
            'dt': 0.05
        }

        args_long_dict = {
            'K_P': 0.2,  # Slightly lower gain for acceleration control
            'K_D': 0.3,  # Moderate derivative gain
            'K_I': 0.01, # Small integral gain
            'dt': 0.05
        }

        PID = VehiclePIDController(vehicle, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)
        return PID

    def find_dist_veh(self, vehicle_loc, target):
        dist = math.sqrt(
            (target.location.x - vehicle_loc.x)**2 +
            (target.location.y - vehicle_loc.y)**2
        )
        return dist

    def drive_through_plan(self, planned_route, vehicle, speed, PID):
        """
        This function drives through the planned_route with the speed passed in the argument
        """
        i = 0
        waypt_cnt = len(planned_route) - 1
        target = planned_route[0]

        cnt = 0
        while True:
            # Update the spectator's view
            self.world.get_spectator().set_transform(
                self.get_transform(vehicle.get_location() + carla.Location(z=1, x=0.5),
                                   vehicle.get_transform().rotation.yaw - 180)
            )

            vehicle_loc = self.odom.location  # Using the odom transform

            distance_v = self.find_dist_veh(vehicle_loc, target)

            control = PID.run_step(speed, target)
            ctrl_msg = self.create_ctrl_msg(control.throttle,
                                            control.steer,
                                            control.brake)
            self.vehicle_control_publisher.publish(ctrl_msg)

            if i == (len(planned_route) - 1):
                print("last waypoint reached")
                break

            if (distance_v < 3.5):
                control = PID.run_step(speed, target)
                ctrl_msg = self.create_ctrl_msg(control.throttle,
                                                control.steer,
                                                control.brake)
                self.vehicle_control_publisher.publish(ctrl_msg)
                i = i + 1
                target = planned_route[i]

            if cnt % 5 == 0:
                print("=----------------------------------------------------------")
                print(f"\n{GREEN} ***** from current loc to {i}/{waypt_cnt} waypoint distance: {distance_v}{RESET}\n")
                print("ROS2 vehicle location: ", self.odom.location)
                print("CARLA vehicle location: ", vehicle.get_location())
                print("target location: ", target.location)
                # Minimal equivalent to rclpy.spin_once(self)
                rospy.sleep(0.001)

            cnt += 1

        control = PID.run_step(0, planned_route[len(planned_route) - 1])
        ctrl_msg = self.create_ctrl_msg(control.throttle,
                                        control.steer,
                                        control.brake)
        self.vehicle_control_publisher.publish(ctrl_msg)

    def run(self):
        desired_velocity = 10  # Km/h

        while not rospy.is_shutdown():
            # Minimal equivalent to rclpy.spin_once
            rospy.sleep(0.001)

            if self.start_pose is None or self.end_pose is None:
                rospy.loginfo(f'Start pose: {self.start_pose}, End pose: {self.end_pose}')

            elif not self.waypoints_list:
                rospy.loginfo('Waiting for waypoints to be generated...')

            else:
                # Delay to ensure waypoints are populated
                rospy.loginfo('Waiting a bit for waypoints to be fully populated...')
                time.sleep(1)  # Add a 1-second delay
                rospy.loginfo(f'Generated {len(self.waypoints_list)} waypoints from start to end pose')

                # calculating life time of the marker
                total_dist = self.find_dist_veh(self.waypoints_list[0].location,
                                                self.waypoints_list[len(self.waypoints_list) - 1])
                marker_life_time = (total_dist / desired_velocity) * 3.6

                # Draw waypoints on the Carla map
                for w in self.waypoints_list:
                    self.world.debug.draw_string(w.location, 'O', draw_shadow=False,
                                                 color=Color(r=255, g=0, b=0), life_time=5000000.0,
                                                 persistent_lines=True)

                # drive the vehicle
                PID = self.setup_PID(self.vehicle)
                self.drive_through_plan(self.waypoints_list, self.vehicle, desired_velocity, PID)

                # After processing, break the loop if needed
                break


def main():
    rospy.init_node('carla_vehicle_control', anonymous=True)
    route_planner = CarlaVehicleControl()
    route_planner.run()
    # Equivalent to rclpy.shutdown()
    rospy.signal_shutdown('Done')


if __name__ == '__main__':
    main()
