#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import rclpy.node
from rclpy.parameter import ParameterValue

from ament_index_python.packages import get_package_share_directory
import os

import yaml

import sys

class ControllerYAMLGenerator(Node):
    def __init__(self, urdf_file, tool):
        super().__init__('controller_yaml_generator')
        self.get_logger().info('Controller YAML Generator Node has been initialized.')

        # Get URDF File Name -
        psm_type = "classic"
        if urdf_file == 'psm.si.urdf.xacro':
            psm_type = "si"
            if tool not in ['P420006', 'SF826001']:
                tool = 'P420006'

        joint_names = ['PSM_outer_yaw', 'PSM_pitch', 'PSM_outer_insertion', 'PSM_outer_roll']
        if psm_type == "si":
            joint_names = ['PSM_yaw', 'PSM_pitch']

        """
        # # Blade
        # - PSM_outer_wrist_pitch
        # - PSM_outer_wrist_yaw
        # - PSM_jaw

        # # Caudier Blade
        # - PSM_outer_wrist_pitch
        # - PSM_outer_wrist_yaw

        # # Caudier
        # - PSM_outer_wrist_pitch
        # - PSM_outer_wrist_yaw
        # - PSM_jaw

        # # SCA Blade
        # - PSM_outer_wrist_pitch
        # - PSM_outer_wrist_yaw

        # # SCA
        # - PSM_outer_wrist_pitch
        # - PSM_outer_wrist_yaw
        # - PSM_jaw

        # # Snake
        # - PSM_outer_wrist_pitch1
        # - PSM_outer_wrist_yaw1
        # - PSM_outer_wrist_yaw2
        # - PSM_outer_wrist_pitch2
        # - PSM_jaw

        # # P420006
        - PSM_insertion
        - PSM_roll
        - PSM_wrist_pitch
        - PSM_wrist_yaw
        - PSM_jaw

        # # SF826001
        - PSM_insertion
        - PSM_roll
        """

        if psm_type == "classic":
            if tool == 'blade':
                joint_names.extend(['PSM_outer_wrist_pitch', 'PSM_outer_wrist_yaw', 'PSM_jaw'])
            elif tool == 'caudier_blade':
                joint_names.extend(['PSM_outer_wrist_pitch', 'PSM_outer_wrist_yaw'])
            elif tool == 'caudier':
                joint_names.extend(['PSM_outer_wrist_pitch', 'PSM_outer_wrist_yaw', 'PSM_jaw'])
            elif tool == 'sca_blade':
                joint_names.extend(['PSM_outer_wrist_pitch', 'PSM_outer_wrist_yaw'])
            elif tool == 'sca':
                joint_names.extend(['PSM_outer_wrist_pitch', 'PSM_outer_wrist_yaw', 'PSM_jaw'])
            elif tool == 'snake':
                joint_names.extend(['PSM_outer_wrist_pitch1', 'PSM_outer_wrist_yaw1', 'PSM_outer_wrist_yaw2', 'PSM_outer_wrist_pitch2', 'PSM_jaw'])
        elif psm_type == "si":
            if tool == 'P420006':
                joint_names.extend(['PSM_insertion', 'PSM_roll', 'PSM_wrist_pitch', 'PSM_wrist_yaw', 'PSM_jaw'])
            elif tool == 'SF826001':
                joint_names.extend(['PSM_insertion', 'PSM_roll'])

        if urdf_file == 'both_psms.classic.urdf.xacro':
            joint_types = ['_outer_yaw', '_pitch', '_outer_insertion', '_outer_roll']
            joint_names = ["one" + joint for joint in joint_types]
            joint_names.extend(["two" + joint for joint in joint_types])

        # Define the updated YAML configuration
        self.yaml = {
                'controller_manager': {
                    'ros__parameters': {
                        'use_sim_time': True,
                        'update_rate': 100,
                        'joint_state_broadcaster': {
                            'type': 'joint_state_broadcaster/JointStateBroadcaster'
                        },
                        'forward_position_controller': {
                            'type': 'position_controllers/JointGroupPositionController'
                        }
                    }
                },
                'forward_position_controller': {
                    'ros__parameters': {
                        'joints': joint_names,
                        'interface_name': 'position'
                    }
                }
            }

    def generate_yaml_file(self):

        # Get the path to the psm_description package
        psm_description_path = get_package_share_directory('psm_description')
        controller_file_path = os.path.join(psm_description_path, 'config', 'psm.controllers.yaml')

        if not os.path.exists(controller_file_path):
            os.makedirs(os.path.dirname(controller_file_path), exist_ok=True)
        else:
            os.remove(controller_file_path)
            self.get_logger().info('Removed the existing psm.controllers.yaml file.')

        # Generate the YAML file
        with open(controller_file_path, 'w') as file:
            yaml.dump(self.yaml, file, sort_keys=False)
        self.get_logger().info('Generated psm.controllers.yaml file.')

def main(args=None):
    rclpy.init(args=args)
    controller_generate = ControllerYAMLGenerator(urdf_file=sys.argv[1], tool=sys.argv[2])
    controller_generate.generate_yaml_file()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
