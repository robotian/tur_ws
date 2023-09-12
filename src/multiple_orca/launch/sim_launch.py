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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    # print("launch start")
    print_cmd = LogInfo(msg="Launch Sequence Start")
    multiorca_dir = get_package_share_directory('multiple_orca')
    # orca_bringup_dir = get_package_share_directory('orca_bringup')
    # orca_description_dir = get_package_share_directory('orca_description')

    ardusub_params_file = os.path.join(multiorca_dir, 'cfg', 'sub.parm')
    mavros_params_file = os.path.join(multiorca_dir, 'params', 'sim_mavros_params.yaml')
    orca_params_file = os.path.join(multiorca_dir, 'params', 'sim_orca_params.yaml')
    # rosbag2_record_qos_file = os.path.join(multiorca_dir, 'params', 'rosbag2_record_qos.yaml')
    rviz_file = os.path.join(multiorca_dir, 'cfg', 'sim_launch.rviz')
    # world_file = os.path.join(multiorca_dir, 'worlds', 'sand.world')

    sim_left_ini = os.path.join(multiorca_dir, 'cfg', 'sim_left.ini')
    sim_right_ini = os.path.join(multiorca_dir, 'cfg', 'sim_right.ini')

    # check if you have all the parameter files and models before changing the namespace
    rov_ns = 'rov1'  

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')] 

    # Start Gazebo with default underwater world
    spawn_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(multiorca_dir, 'launch', 'start_gz_sim.launch.py'))                       
        )
    set_use_sim_time_param = SetParameter(name='use_sim_time', value=True)
    # spawn rov sdf model
    sdf_filepath = os.path.join(multiorca_dir, 'models', rov_ns, 'model.sdf')   
    opt_str = ['sdf_filename:"{name}"'.format(name=sdf_filepath),
                   'name:"{rov_name}"'.format(rov_name=rov_ns), 
                   'pose:{position:{',
                   'x:', '0.0',
                   ',y:', '0.0',
                   ',z:', '0.0',r'},',
                   'orientation:{',
                   'x:', '0.0',
                   ',y:', '0.0',
                   ',z:', '0.0',
                   ',w:', '1.0',r'}}']
    start_spawn_rov_cmd =  ExecuteProcess(
            cmd=['gz', 'service', '-s', '/world/sand/create', '--reqtype', 'gz.msgs.EntityFactory',
                 '--reptype','gz.msgs.Boolean','--timeout','1000','--req',opt_str],                 
            output='screen')    
     
    # Gazebo Bridge.
    start_gz_brdg_cmd = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"gz_brdg_{rov_ns}", #  "gz_brg_rov1",
            # namespace=rov_ns,  # not directly using Namespace
            parameters=[
                {
                    "config_file": os.path.join(
                        multiorca_dir, "cfg", f"{rov_ns}_bridge.yaml" 
                    ),
                    "qos_overrides./tf_static.publisher.durability": "transient_local",
                }
            ],
            remappings=remappings,
            output="screen",
        )
    
    # Get images from Gazebo Sim to ROS
    start_img_brdg_node_cmd = Node(
            package='ros_gz_image',
            executable='image_bridge',
            name=f"gz_im_brdg_{rov_ns}", #name="gz_img_brg_rov1",
            # arguments=['rov1/stereo_left', 'rov1/stereo_right'],  # need to add more image topics as the number of ROVs increases. NO namespace required            
            arguments=[f"{rov_ns}/stereo_left", f"{rov_ns}/stereo_right"],
            remappings=remappings,
            output='screen',
        )
    

    return LaunchDescription([
        print_cmd,
        set_use_sim_time_param,
        DeclareLaunchArgument(
            'namespace',
            default_value=rov_ns,
            description='Define namespace'
        ),
        DeclareLaunchArgument(
            'ardusub',
            default_value='True',
            description='Launch ArduSUB with SIM_JSON?'
        ),

        DeclareLaunchArgument(
            'bag',
            default_value='False',
            description='Bag interesting topics?',
        ),

        DeclareLaunchArgument(
            'base',
            default_value='True',
            description='Launch base controller?',
        ),

        DeclareLaunchArgument(
            'gzclient',
            default_value='True',
            description='Launch Gazebo UI?'
        ),

        DeclareLaunchArgument(
            'mavros',
            default_value='True',
            description='Launch mavros?',
        ),

        DeclareLaunchArgument(
            'nav',
            default_value='True',
            description='Launch navigation?',
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='True',
            description='Launch rviz?',
        ),

        DeclareLaunchArgument(
            'slam',
            default_value='True',
            description='Launch SLAM?',
        ),

        # Bag useful topics
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'bag', 'record',
        #         '--qos-profile-overrides-path', rosbag2_record_qos_file,
        #         '--include-hidden-topics',
        #         '/cmd_vel',
        #         '/mavros/local_position/pose',
        #         '/mavros/rc/override',
        #         '/mavros/setpoint_position/global',
        #         '/mavros/state',
        #         '/mavros/vision_pose/pose',
        #         '/model/orca4/odometry',
        #         '/motion',
        #         '/odom',
        #         '/orb_slam2_stereo_node/pose',
        #         '/orb_slam2_stereo_node/status',
        #         '/pid_z',
        #         '/rosout',
        #         '/tf',
        #         '/tf_static',
        #     ],
        #     output='screen',
        #     condition=IfCondition(LaunchConfiguration('bag')),
        # ),

        # Launch rviz
        # (TODO MK) need to create rviz config for multiple robots in different namespaces
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_file,'--ros-args','-r','/tf:=/rov1/tf','-r','/tf_static:=/rov1/tf_static'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),

        # Launch ArduSub w/ SIM_JSON (wipe eeprom w/ -w)
        # ardusub must be on the $PATH, see src/orca4/setup.bash
        ExecuteProcess(
            cmd=['ardusub', '-S', '-w', '-M', 'JSON', '--defaults', ardusub_params_file,
                 '-I0', '--home', '33.810313,-118.39386700000001,0.0,270.0'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('ardusub')),
        ),

        spawn_world_cmd,
        start_spawn_rov_cmd,
        start_img_brdg_node_cmd,

        # Gazebo Sim doesn't publish camera info, so do that here
        Node(
            package='orca_base',
            executable='camera_info_publisher',
            name='left_info_publisher',
            output='screen',
            namespace=rov_ns,
            parameters=[{
                'camera_info_url': 'file://' + sim_left_ini,
                'camera_name': 'stereo_left',
                'frame_id': 'stereo_left_frame',
                'timer_period_ms': 50,
            }],
            remappings=[
                ('camera_info', 'stereo_left/camera_info'),
                tuple(remappings),
            ],
        ),

        Node(
            package='orca_base',
            executable='camera_info_publisher',
            name='right_info_publisher',
            output='screen',
            namespace=rov_ns,
            parameters=[{
                'camera_info_url': 'file://' + sim_right_ini,
                'camera_name': 'stereo_right',
                'frame_id': 'stereo_right_frame',
                'timer_period_ms': 50,
            }],
            remappings=[
                ('camera_info', 'stereo_right/camera_info'),
                tuple(remappings),
            ],
        ),

        start_gz_brdg_cmd,        

        # Bring up Orca and Nav2 nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(multiorca_dir, 'launch', 'bringup.py')),
            launch_arguments={
                'namespace':rov_ns,
                # 'namespace':LaunchConfiguration('namespace'),
                'base': LaunchConfiguration('base'),
                'mavros_ns':f"{rov_ns}/mavros",
                'mavros': LaunchConfiguration('mavros'),
                'mavros_params_file': mavros_params_file,
                'nav': LaunchConfiguration('nav'),
                'orca_params_file': orca_params_file,
                'slam': LaunchConfiguration('slam'),
            }.items(),
        ),
    ])

