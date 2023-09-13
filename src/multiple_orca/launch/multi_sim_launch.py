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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # define variables that are not depending on the namespace
    multiorca_dir = get_package_share_directory('multiple_orca')  
    
    orca_params_file = os.path.join(multiorca_dir, 'params', 'sim_orca_params.yaml')

    sim_left_ini = os.path.join(multiorca_dir, 'cfg', 'sim_left.ini')
    sim_right_ini = os.path.join(multiorca_dir, 'cfg', 'sim_right.ini')

    
    


    # to bring up a rov, there should exit configuration files for the rov.
    # cfg/{rov-name}_bridge.yaml
    # models/{rov-name}
    # params/sim_mavros_params_{rov-name}
    robots = [
        {'name': 'tur1', 'x_pose': 0.0, 'y_pose': 0.0, 'z_pose': 0.0,
                           'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0, 'instance':'-I0',
                           'home':'33.810313,-118.39386700000001,0.0,270.0',
                           'sysid':'1',
                           'fcu_url':'tcp://localhost:5760',
                           'gcs_url':'udp://@localhost:14550'},
        # {'name': 'rov2', 'x_pose': 0.0, 'y_pose': 3.0, 'z_pose': 0.0,
        #                    'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0, 'instance':'-I1',
        #                    'home':'33.810311,-118.39386700000001,0.0,270.0',
        #                    'sysid':'2',
        #                    'fcu_url':'tcp://localhost:5770',
        #                    'gcs_url':'udp://@localhost:14560'},
        # {'name': 'rov3', 'x_pose': 0.0, 'y_pose': 6.0, 'z_pose': 0.0,
        #                    'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0, 'instance':'-I2',
        #                    'home':'33.810311,-118.39386700000001,0.0,270.0',
        #                    'sysid':'3',
        #                    'fcu_url':'tcp://localhost:5780',
        #                    'gcs_url':'udp://@localhost:14570'},
        ]
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')] 
    

    use_sim_time = LaunchConfiguration('use_sim_time')   
    # to use the gazebo time, /clock topic should be published. You can do it using parameter bride.
    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='use_sim_time param')

    # Start Gazebo with default underwater world
    spawn_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(multiorca_dir, 'launch', 'start_gz_sim.launch.py')),
        launch_arguments={'use_sim_time':use_sim_time}.items()
        )   
    
    
    instances_cmds = []
    for robot in robots:        
        rov_ns = robot['name']

        mavros_params_file = os.path.join(multiorca_dir, 'params', f"sim_mavros_params_{robot['name']}.yaml")
        ardusub_rov_params_file = os.path.join(multiorca_dir, 'cfg',  'sub.parm')  

        rviz_config_file = LaunchConfiguration('rviz_config')   

        declare_rviz_config_file_cmd = DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(multiorca_dir, 'cfg', 'nav2_namespaced_view.rviz'),
            description='Full path to the RVIZ config file to use.')
        
        declare_arg_base_cmd = DeclareLaunchArgument(
            'base',
            default_value='True',
            description='Launch base controller?',
        )

        declare_arg_mavros_cmd = DeclareLaunchArgument(
            'mavros',
            default_value='True',
            description='Launch mavros?',
        )

        declare_arg_nav_cmd = DeclareLaunchArgument(
            'nav',
            default_value='True',
            description='Launch navigation?',
        )

        declare_arg_slam_cmd = DeclareLaunchArgument(
            'slam',
            default_value='True',
            description='Launch SLAM?',
        )   

        declare_rviz_cmd = DeclareLaunchArgument(
            'rviz',
            default_value='True',
            description='Launch rviz?',
        )     

        # currently world frames are being published in each rov namespace. 
        # TODO need to make it globally available
        tf_static_pub_cmd = ComposableNodeContainer(
            name='tf_container',
            package='rclcpp_components',
            executable='component_container',  
            namespace='',          
            composable_node_descriptions=[
                ComposableNode(
                    package='tf2_ros',
                    plugin='tf2_ros::StaticTransformBroadcasterNode',
                    name='static_tf_world2map',
                    namespace=rov_ns,
                    parameters=[{
                        'frame_id':'world',
                        'child_frame_id':'map',
                        'translation.x': robot['x_pose'],
                        'translation.y': robot['y_pose'],
                        'translation.z': robot['z_pose'],
                        'rotation.x': robot['x'],
                        'rotation.y': robot['y'],
                        'rotation.z': robot['z'],
                        'rotation.w': robot['w']
                        },
                        {'use_sim_time':use_sim_time}],
                    remappings=remappings)
            ],
            output='screen'
        )

        # Launch ArduSub w/ SIM_JSON (wipe eeprom w/ -w)
        # ardusub must be on the $PATH, see src/orca4/setup.bash
        # ardusub options
        # --help|-h                display this help information
        # --wipe|-w                wipe eeprom
        # --unhide-groups|-u       parameter enumeration ignores AP_PARAM_FLAG_ENABLE
        # --speedup|-s SPEEDUP     set simulation speedup
        # --rate|-r RATE           set SITL framerate
        # --console|-C             use console instead of TCP ports
        # --instance|-I N          set instance of SITL (adds 10*instance to all port numbers)
        # --synthetic-clock|-S     set synthetic clock mode
        # --home|-O HOME           set start location (lat,lng,alt,yaw) or location name
        # --model|-M MODEL         set simulation model
        # --config string          set additional simulation config string
        # --fg|-F ADDRESS          set Flight Gear view address, defaults to 127.0.0.1
        # --disable-fgview         disable Flight Gear view
        # --gimbal                 enable simulated MAVLink gimbal
        # --autotest-dir DIR       set directory for additional files
        # --defaults path          set path to defaults file
        # --rtscts                 enable rtscts on serial ports (default false)
        # --base-port PORT         set port num for base port(default 5670) must be before -I option
        # --rc-in-port PORT        set port num for rc in
        # --sim-address ADDR       set address string for simulator
        # --sim-port-in PORT       set port num for simulator in
        # --sim-port-out PORT      set port num for simulator out
        # --irlock-port PORT       set port num for irlock
        # --start-time TIMESTR     set simulation start time in UNIX timestamp
        # --sysid ID               set SYSID_THISMAV
        # --slave number           set the number of JSON slaves

        start_ardusub_rov_cmd = ExecuteProcess(
                cmd=['ardusub', '-S', '-w', '-M', 'JSON', '--defaults', ardusub_rov_params_file,
                    robot['instance'], '--home', robot['home'],'--sysid',robot['sysid']], 
                output='screen')

        
        # spawn rov sdf model        
        sdf_filepath = os.path.join(multiorca_dir, 'models', rov_ns, 'model.sdf')   
        opt_str = ['sdf_filename:"{name}"'.format(name=sdf_filepath),
                    'name:"{rov_name}"'.format(rov_name=rov_ns), 
                    'pose:{position:{',
                    'x:', str(robot['x_pose']),
                    ',y:', str(robot['y_pose']),
                    ',z:', str(robot['z_pose']),r'},',
                    'orientation:{',
                    'x:', str(robot['x']),
                    ',y:', str(robot['y']),
                    ',z:', str(robot['z']),
                    ',w:', str(robot['w']),r'}}']        
        
        # To spawn a model, you need to wait until the 'create' service becomes available. 
        # But there is no way to check its availability. So, give it enough time to wait by increasing 'timeout'
        # If it does not work, then you can spawn them seperately using the following command line inputs. 
        #gz service -s /world/sand/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename:"/home/robotian/orca_ws/src/multiple_orca/models/rov1/model.sdf" pose:{position:{x:0,y:0,z:0},orientation:{x:0.0,y:0.0,z:0.0,w:1.0}}'
        #gz service -s /world/sand/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename:"/home/robotian/orca_ws/src/multiple_orca/models/rov2/model.sdf" pose:{position:{x:0,y:3,z:0},orientation:{x:0.0,y:0.0,z:0.0,w:1.0}}'
        start_spawn_rov_cmd =  ExecuteProcess(
                cmd=['gz', 'service', '-s', '/world/sand/create', '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype','gz.msgs.Boolean','--timeout','20000','--req',opt_str],                 
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
                    },
                    {'use_sim_time':use_sim_time}
                ],
                # remappings=remappings,
                output="screen",
            )
        
        # Get images from Gazebo Sim to ROS
        start_img_brdg_node_cmd = Node(
                package='ros_gz_image',
                executable='image_bridge',
                name=TextSubstitution(text=f"gz_im_brdg_{rov_ns}"), #name="gz_img_brg_rov1",
                arguments=[f"{rov_ns}/stereo_left", f"{rov_ns}/stereo_right"],
                # remappings=remappings,
                parameters=[{'use_sim_time':use_sim_time}],
                output='screen',
            )        
        
        start_rviz_cmd = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(multiorca_dir,'launch', 'rviz_launch.py')),
                    launch_arguments={'namespace': TextSubstitution(text=rov_ns),
                                    'use_namespace': 'True',
                                    'rviz_config': rviz_config_file,
                                    'use_sim_time':use_sim_time}.items(),
                    condition=IfCondition(LaunchConfiguration('rviz')))
        
        # Gazebo Sim doesn't publish camera info, so do that here
        start_orca_cam_left_cmd = Node(
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
                'use_sim_time':use_sim_time
            }],
            remappings=[
                ('camera_info', 'stereo_left/camera_info'),
                tuple(remappings),
            ],
        )

        start_orca_cam_right_cmd = Node(
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
                'use_sim_time':use_sim_time
            }],
            remappings=[
                ('camera_info', 'stereo_right/camera_info'),
                tuple(remappings),
            ],
        )
        

        # Bring up Orca and Nav2 nodes
        orca_bringup_cmd = IncludeLaunchDescription(
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
                'use_sim_time':use_sim_time
            }.items(),
        )       


        instances_cmds.append(start_ardusub_rov_cmd)
        instances_cmds.append(declare_rviz_config_file_cmd)
        instances_cmds.append(declare_arg_base_cmd)
        instances_cmds.append(declare_arg_mavros_cmd)
        instances_cmds.append(declare_arg_nav_cmd)
        instances_cmds.append(declare_arg_slam_cmd)   
        instances_cmds.append(declare_rviz_cmd)                
        instances_cmds.append(start_gz_brdg_cmd)
        instances_cmds.append(start_img_brdg_node_cmd)
        instances_cmds.append(start_rviz_cmd)
        instances_cmds.append(start_orca_cam_left_cmd)
        instances_cmds.append(start_orca_cam_right_cmd)        
        instances_cmds.append(orca_bringup_cmd)
        instances_cmds.append(start_spawn_rov_cmd)
        instances_cmds.append(tf_static_pub_cmd)

    
    ld = LaunchDescription()
        
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(spawn_world_cmd)

    for inst in instances_cmds:
        ld.add_action(inst)
    
    return ld

