#!/usr/bin/env python3

import numpy as np
import gtsam

from functools import partial
from typing import List, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Twist, Point, Quaternion
from tf2_ros import TransformException, Buffer, TransformListener
import tf2_geometry_msgs

def pos_error(goal: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[List[np.ndarray]]) -> np.ndarray:

    pos_key = this.keys()[0]
    pos = values.atVector(pos_key)

    error = pos - goal
    if jacobians is not None:
        jacobians[0] = np.eye(1)

    return error

def dyn_error(dt: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[List[np.ndarray]]) -> np.ndarray:

    pos_key = this.keys()[0]
    pos = values.atVector(pos_key)

    last_pos_key = this.keys()[1]
    last_pos = values.atVector(last_pos_key)

    cmd_vel_key = this.keys()[2]
    cmd_vel = values.atVector(cmd_vel_key)


    error = pos - last_pos - cmd_vel*dt
    if jacobians is not None:
        jacobians[0] = np.eye(1)
        jacobians[1] = -np.eye(1)
        jacobians[2] = -np.eye(1)

    return error


class FormationControlNode(Node):
    def __init__(self):
        super().__init__('formation_control_node')

        # Member variable containing target poses in vehicle_1's frame
        self.target_pose_dict = {
            'vehicle_2': Pose(position=Point(x=np.float64(-1.0), y=np.float64(1.0))),
            'vehicle_3': Pose(position=Point(x=np.float64(-1.0), y=np.float64(1.0))),
            'vehicle_4': Pose(position=Point(x=np.float64(-2.0), y=np.float64(2.0)))
        }

        self.index_offset_dict = {
            'vehicle_2': 2000,
            'vehicle_3': 3000,
            'vehicle_4': 4000
        }

        # Publishers for each vehicle's command velocities
        self.cmd_vel_pubs = {
            'vehicle_2': self.create_publisher(Twist, '/model/vehicle_2/cmd_vel', 10),
            'vehicle_3': self.create_publisher(Twist, '/model/vehicle_3/cmd_vel', 10),
            'vehicle_4': self.create_publisher(Twist, '/model/vehicle_4/cmd_vel', 10)
        }

        # tf2 buffer and listener
        self.tf_buffer = Buffer(rclpy.duration.Duration(seconds=1.))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer for main control loop
        self.delta_t = 0.1
        self.timer = self.create_timer(self.delta_t, self.control_loop)

        # Variables for factor graph
        self.n_lookahead_steps = 10
        self.pos_noise = gtsam.noiseModel.Isotropic.Sigma(1, 1.0)
        self.cmd_vel_noise = gtsam.noiseModel.Isotropic.Sigma(1, 10.0)

    def control_loop(self):
        self.get_logger().info("Control callback")
        """
        Main control loop to compute and send commands to the team of vehicles.
        """

        # Create nonlinear factor graph and values container
        self.graph = gtsam.NonlinearFactorGraph()
        self.values = gtsam.Values()

        # Initialize optimizer
        self.params = gtsam.GaussNewtonParams()
        self.optimizer = gtsam.GaussNewtonOptimizer(self.graph, self.values, self.params)

        for vehicle_id in ['vehicle_2', 'vehicle_3', 'vehicle_4']:
            try:
                # Get the target pose in vehicle_1's frame
                target_pose = self.target_pose_dict[vehicle_id]

                # Convert target pose to vehicle_n's frame
                target_pose_stamped = PoseStamped()
                target_pose_stamped.header.frame_id = 'vehicle_1'
                target_pose_stamped.header.stamp = rclpy.time.Time().to_msg()
                target_pose_stamped.pose = target_pose

                # Transform pose to vehicle_n's frame
                transformed_pose = self.tf_buffer.transform(
                    target_pose_stamped,
                    vehicle_id
                    # timeout=rclpy.duration.Duration(seconds=0.1)
                )

                # Placeholder: Compute control input
                self.get_logger().info("Vehicle %s target pose (robot frame): %s" % (vehicle_id, transformed_pose))

                for graph_idx in range(self.n_lookahead_steps):

                    # Add variables
                    pos_var = gtsam.symbol('p',self.index_offset_dict[vehicle_id] + graph_idx)
                    cmd_vel_var = gtsam.symbol('u',self.index_offset_dict[vehicle_id] + graph_idx)
                    # self.values.insert() # TODO insert 0 pose estimate
                    
                    # Add navigation goal factor
                    nav_factor = gtsam.CustomFactor(self.pos_noise, [pos_var],partial(pos_error, np.array([transformed_pose.pose.position.x])))
                    self.graph.add(nav_factor)

                    # Add dynamics constraint
                    if graph_idx>0:
                        dynamics_factor = gtsam.CustomFactor(self.pos_noise, [pos_var, last_pos_var, cmd_vel_var], partial(dyn_error, np.array([self.delta_t])))
                        self.graph.add(dynamics_factor)

                    # Add input penalty
                    # TODO add prior cmd_vel factor

                    last_pos_var = pos_var

                # Add prior constraint
                # TODO -  add prior pose factor

            except TransformException as ex:
                self.get_logger().error(
                    f"Could not transform target pose for {vehicle_id}: {ex}"
                )

        # Solve graph
        self.result = self.optimizer.optimize()

        # Extract optimal control inputs

        # Publish optimal control inputs

        self.get_logger().info("")

    def compute_control_input(self, pose):
        """
        Placeholder function to compute control input from a Pose.
        Replace this with your actual control logic.
        """
        twist = Twist()
        # Example: Dummy zero control input
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        return twist


def main(args=None):
    rclpy.init(args=args)
    node = FormationControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()