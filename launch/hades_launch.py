import os 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    mavros_pkg_share = FindPackageShare(package='mavros').find('mavros')
    hades_lander_pkg_share = FindPackageShare(package='hades_lander').find('hades_lander')
    mavros_launch_file = os.path.join(mavros_pkg_share, 'launch', 'node.launch')
    
    # Define the path to your config file
    mavros_config_file = os.path.join(hades_lander_pkg_share, 'config', 'mavros_config.yaml')
    pluginlists_yaml_file = os.path.join(mavros_pkg_share, 'launch', 'apm_pluginlists.yaml')
    # Create the IncludeLaunchDescription action
    mavros_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(mavros_launch_file),
        # Pass the fcu_url as a launch argument
        launch_arguments={
            'fcu_url': '/dev/ttyACM0:57600',
            'gcs_url': '',
            'log_level': 'ERROR',
            'tgt_system': '1',
            'tgt_component': '1',
            'config_yaml': mavros_config_file,
            'pluginlists_yaml': pluginlists_yaml_file 
        }.items()
    )
    


    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        parameters=[
            {'video_device': '/dev/video4'}, 
            {'image_size': [640, 480]},
        ],
        remappings=[
            ('image_raw', '/camera/image_raw')
        ]
    )
    
    # 2. Start the Vision Node
    vision_node = Node(
        package='hades_lander',
        executable='vision_node',  
        name='vision_node',
        output='screen',
        parameters=[
            {'hades_vision.save_detections': True},
            {'hades_vision.save_path': '/home/nyx/detection_summary'},
            {'hades_vision.save_mode': 'interval'},
            {'hades_vision.save_interval_sec': 0.5}
        ]
    )
    

    lander_node = Node(
        package='hades_lander',
        executable='bullseye_node', 
        name='bullseye_lander',
        output='screen'
    )

    return LaunchDescription([
        mavros_node,  
        camera_node,
        vision_node,
        lander_node
    ])