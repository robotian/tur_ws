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
Bring up all nodes

Use a modified navigation_launch.py that doesn't launch velocity_smoother.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import TextSubstitution
from launch.launch_context import LaunchContext

from launch_ros.actions import SetParameter
# import yaml

def launch_setup(context, *args, **kwargs):
    arg_namespace = context.perform_substitution(LaunchConfiguration('namespace'))
    mavros_params_file = LaunchConfiguration('mavros_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_mav_node = Node(
                package='mavros',
                executable='mavros_node',
                output='screen',
                # mavros_node is actually many nodes, so we can't override the name
                # name='mavros_node',
                # namespace='rov1/mavros',
                namespace=f"{arg_namespace}/mavros",
                parameters=[mavros_params_file, {'use_sim_time': use_sim_time}],            
                remappings=[('/tf', f"/{arg_namespace}/tf"),
                    ('/tf_static', f"/{arg_namespace}/tf_static")] ,
                condition=IfCondition(LaunchConfiguration('mavros')),
            )
    return [start_mav_node]



def generate_launch_description():
    
    print_cmd = LogInfo(msg="bringup Launch Start")
    multiorca_dir = get_package_share_directory('multiple_orca')
    nav2_bt_file = os.path.join(multiorca_dir, 'behavior_trees', 'orca4_bt.xml')
    nav2_params_file = os.path.join(multiorca_dir, 'params', 'nav2_params.yaml')


    mavros_params_file = LaunchConfiguration('mavros_params_file')
    orca_params_file = LaunchConfiguration('orca_params_file')

    
    # get_package_share_directory('orb_slam2_ros') will fail if orb_slam2_ros isn't installed
    orb_voc_file = os.path.join('install', 'orb_slam2_ros', 'share', 'orb_slam2_ros',
                                'orb_slam2', 'Vocabulary', 'ORBvoc.txt')

    # Rewrite to add the full path
    # The rewriter will only rewrite existing keys
    # check https://github.com/ros-planning/navigation2/issues/2117 to understand how RewrittenYaml 
    # works. Simply speaking, it replaces some parameter values from 'source_file' with given 'param_rewrites' values

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',   
        param_rewrites={
            'default_nav_to_pose_bt_xml': nav2_bt_file,
        },
        convert_types=True)   
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')] 
    
    # for debugging start
    # context = LaunchContext()
    # new_yaml = configured_nav2_params.perform(context)
    # # with open(nav2_params_file) as file:
    # #     list = yaml.load(file, Loader=yaml.FullLoader)
    # #     print ("Orig: " + str(list['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']))

    # # with open(new_yaml) as file:
    # #     list = yaml.load(file, Loader=yaml.FullLoader)
    # #     print ("New: " + str(list['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']))    

    # with open(new_yaml) as file:
    #     list = yaml.load(file, Loader=yaml.FullLoader)   
    #     print("Key: Value")
    #     for key, value in list.items():
    #         print(f"{key}: {value}")   

    # input("Press Enter to continue...")
    # for debugging end
    # set_use_sim_time_param = SetParameter(name='use_sim_time', value=True)
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # mavros_ns = LaunchConfiguration('mavros_ns')
    

    return LaunchDescription([
        print_cmd,
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'namespace',
            default_value=TextSubstitution(text=""),
            description='Top-level namespace'
        ),

        DeclareLaunchArgument(
            'base',
            default_value=TextSubstitution(text="True"),
            description='Launch base controller?',
        ),

        DeclareLaunchArgument(
            'mavros',
            default_value='True',
            description='Launch mavros?',
        ),
        
        DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'mavros_params_file',
            default_value=os.path.join(multiorca_dir, 'params', 'sim_mavros_params.yaml'),
            description='Full path to the ROS2 parameters file to use for mavros nodes',
        ),

        DeclareLaunchArgument(
            'nav',
            default_value='True',
            description='Launch navigation?',
        ),

        DeclareLaunchArgument(
            'orca_params_file',
            default_value=os.path.join(multiorca_dir, 'params', 'orca_params.yaml'),
            description='Full path to the ROS2 parameters file to use for Orca nodes',
        ),

        DeclareLaunchArgument(
            'slam',
            default_value='True',
            description='Launch SLAM?',
        ),
        DeclareLaunchArgument(
            'mavros_ns',
            default_value='',
            description='Separate namespace for mavros'
        ),

        OpaqueFunction(function=launch_setup),  # mavros node
        

        # Translate messages MAV <-> ROS
        # Node(
        #     package='mavros',
        #     executable='mavros_node',
        #     output='screen',
        #     # mavros_node is actually many nodes, so we can't override the name
        #     # name='mavros_node',
        #     # namespace='rov1/mavros',
        #     namespace=mavros_ns,
        #     parameters=[mavros_params_file],            
        #     remappings=[('/tf', '/rov2/tf'),
        #           ('/tf_static', '/rov2/tf_static')] ,
        #     # remappings=[('/tf', '/{}/tf'.format(LaunchConfiguration('namespace'))),
        #     #       ('/tf_static', '/{}/tf_static'.format(LaunchConfiguration('namespace')))] ,
        #     condition=IfCondition(LaunchConfiguration('mavros')),
        # ),

        

        # Replacement for base_controller: complete the tf tree
        # if base_controller runs, these three static_transfor_publisher will not run. 
        # these are for the case that base_controller does not run. 
        # ExecuteProcess(
        #     cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
        #          '--frame-id', 'map',
        #          '--child-frame-id', 'slam'],
        #     output='screen',
        #     condition=UnlessCondition(LaunchConfiguration('base')),
        # ),

        # ExecuteProcess(
        #     cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
        #          '--frame-id', 'map',
        #          '--child-frame-id', 'odom'],
        #     output='screen',
        #     condition=UnlessCondition(LaunchConfiguration('base')),
        # ),

        # ExecuteProcess(
        #     cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
        #          '--frame-id', 'odom',
        #          '--child-frame-id', 'base_link'],
        #     output='screen',
        #     condition=UnlessCondition(LaunchConfiguration('base')),
        # ),

        # on the other hand, these two static tf pub need to be running
        # Replacement for an URDF file: base_link->left_camera_link is static
        # ExecuteProcess(
        #     cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
        #          '--x', '-0.15',
        #          '--y', '0.18',
        #          '--z', '-0.0675',
        #          '--pitch', str(math.pi/2),
        #          '--frame-id', 'base_link',
        #          '--child-frame-id', 'left_camera_link',
        #          ],
        #     output='screen',
        # ),

        ComposableNodeContainer(
            name='tf_container',
            package='rclcpp_components',
            executable='component_container',  
            namespace='',          
            composable_node_descriptions=[
                ComposableNode(
                    package='tf2_ros',
                    plugin='tf2_ros::StaticTransformBroadcasterNode',
                    name='static_tf_base2cam',
                    namespace=namespace,
                    parameters=[{
                        'frame_id':'base_link',
                        'child_frame_id':'left_camera_link',
                        'translation.x': -0.15,
                        'translation.y': 0.18,
                        'translation.z': -0.0675,
                        'rotation.x': 0.0,
                        'rotation.y': 0.7071067811865475,
                        'rotation.z': 0.0,
                        'rotation.w': 0.7071067811865476,
                        'use_sim_time': use_sim_time
                        }],
                    remappings=remappings),
                ComposableNode(
                    package='tf2_ros',
                    plugin='tf2_ros::StaticTransformBroadcasterNode',
                    name='static_tf_slam2down',
                    namespace=namespace,
                    parameters=[{
                        'frame_id':'slam',
                        'child_frame_id':'down',
                        'translation.x': 0.0,
                        'translation.y': 0.0,
                        'translation.z': 0.0,
                        'rotation.x': 0.0,
                        'rotation.y': 0.7071067811865475,
                        'rotation.z': 0.0,
                        'rotation.w': 0.7071067811865476,
                        'use_sim_time': use_sim_time
                        }],
                    remappings=remappings
                    )
            ],
            output='screen',
        ),

        # Provide down frame to accommodate down-facing cameras
        # ExecuteProcess(
        #     cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
        #          '--pitch', str(math.pi/2),
        #          '--frame-id', 'slam',
        #          '--child-frame-id', 'down', 
        #          '--ros-args',
        #          '-r','/tf:=/rov2/tf',
        #          '-r','/tf_static:=/rov2/tf_static',
        #          '-r','__ns:=/rov2'],
        #     output='screen',
        # ),



        # orb_slam2: build a map of 3d points, localize against the map, and publish the camera pose
        Node(
            package='orb_slam2_ros',
            executable='orb_slam2_ros_stereo',
            output='screen',
            name='orb_slam2_stereo',
            namespace = namespace,
            parameters=[orca_params_file, {
                'voc_file': orb_voc_file,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('/image_left/image_color_rect', 'stereo_left'),
                ('/image_right/image_color_rect', 'stereo_right'),
                ('/camera/camera_info', 'stereo_right/camera_info'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ],
            condition=IfCondition(LaunchConfiguration('slam')),
        ),
        # Manage overall system (start, stop, etc.)
        Node(
            package='orca_base',
            executable='manager',
            output='screen',
            name='manager',
            parameters=[orca_params_file, 
                        {'use_sim_time': use_sim_time}],
            namespace = namespace,
            remappings=[
                # Topic is hard coded in orb_slam2_ros to /orb_slam2_stereo_node/pose
                ('camera_pose', 'orb_slam2_stereo_node/pose'), 
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ],
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        # Base controller and localizer; manage external nav input, publish tf2 transforms, etc.
        Node(
            package='orca_base',
            executable='base_controller',
            output='screen',
            name='base_controller',
            parameters=[orca_params_file,
                        {'use_sim_time': use_sim_time}],
            namespace = namespace,
            remappings=[
                # Topic is hard coded in orb_slam2_ros to /orb_slam2_stereo_node/pose
                ('camera_pose', 'orb_slam2_stereo_node/pose'),
                # remappings,
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ],
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        # Include the rest of Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(multiorca_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'autostart': 'False',
                'params_file': configured_nav2_params,
                'use_composition': 'False',
                'use_respawn': 'False',
                'container_name': 'nav2_container',
                # 'remappings':remappings,
            }.items(),
            condition=IfCondition(LaunchConfiguration('nav')),
        ),
    ])
