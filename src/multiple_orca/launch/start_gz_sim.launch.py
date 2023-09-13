#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2022 Clyde McQueen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Launch a simulation.

Includes Gazebo, ArduSub, RViz, mavros, all ROS nodes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter

# Load Gazebo world model and run ArduSub plugin

def generate_launch_description():
    multiorca_dir = get_package_share_directory('multiple_orca')
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")        

    
    use_sim_time = LaunchConfiguration('use_sim_time')   
    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='use_sim_time param')
    # Gazebo.
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-v3 -r -s "   # -v: option is for verbose level
            + os.path.join(multiorca_dir, 'worlds', 'sand_empty.world'),
            "use_sim_time":use_sim_time
        }.items(),
    )
    
    # gz_sim_gui = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
    #     ),
    #     launch_arguments={"gz_args": "-v3 -g"}.items(),
    # )
    # set_use_sim_time_param = SetParameter(name='use_sim_time', value=True)
   

    ld = LaunchDescription()    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(gz_sim_server)
    # ld.add_action(gz_sim_gui)


    return ld



