#!/usr/bin/env python3

import os

from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory
from nav2_common.launch import ReplaceString
from launch.substitutions import LaunchConfiguration

declare_namespace_cmd = DeclareLaunchArgument(
    "namespace",
    description='Machine name',
    # default_value='slp_14h_001',
    )

declare_use_sim_time_cmd = DeclareLaunchArgument(
    'use_sim_time',
    description='Use simulation (Gazebo) clock if true',
    # default_value='true',
    )

declare_core_params_cmd = DeclareLaunchArgument(
    'core_params',
    description='Core parameters file path.',
    default_value=[
        os.path.join(
            get_package_share_directory('ks_launchers'),
            'config/global',
            'ks_global_core.yaml'
        ),
    ]
)

declare_simulation_params_cmd = DeclareLaunchArgument(
    'simulation_params',
    description='Simulation parameters file path.',
    default_value=[
        os.path.join(
            get_package_share_directory('ks_launchers'),
            'config/global',
            'ks_global_simulation.yaml'
        ),
    ]
)

declare_real_params_cmd = DeclareLaunchArgument(
    'real_params',
    description='Real parameters file path.',
    default_value=[
        os.path.join(
            get_package_share_directory('vehicle_config'),
            'amtc_config',
            'ks_global_real.yaml'
        ),
    ]
)

declare_machine_params_cmd = DeclareLaunchArgument(
    'machine_params',
    description='Machine parameters file path.',
    default_value=[
        os.path.join(
            get_package_share_directory('ks_launchers'),
            'config/global',
            'ks_global_machine.yaml'
        ),
    ]
)

core_params = ReplaceString(
    source_file=LaunchConfiguration('core_params'),
    replacements={'<robot_ns_f>': (LaunchConfiguration('namespace'))},
)
simulation_params = ReplaceString(
    source_file=LaunchConfiguration('simulation_params'),
    replacements={'<robot_ns_f>': (LaunchConfiguration('namespace'))},
)
real_params = ReplaceString(
    source_file=LaunchConfiguration('real_params'),
    replacements={'<robot_ns_f>': (LaunchConfiguration('namespace'))},
)
machine_params = ReplaceString(
    source_file=LaunchConfiguration('machine_params'),
    replacements={'<robot_ns_f>': (LaunchConfiguration('namespace'))},
)
