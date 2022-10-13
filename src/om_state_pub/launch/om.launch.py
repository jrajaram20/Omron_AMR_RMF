from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    action_serve = Node(
        package='om_aiv_navigation',
        executable='action_server',
        #name = 'action_server',
        #output='screen',
        parameters=[{
            'ip_address': "192.168.250.154",
            'port': 7171,
            'arcl_passwd': "omron"
        }]
    )
    action_serve2 = Node(
        package='om_aiv_navigation',
        executable='action_server',
        #name = 'action_server',
        output='screen',
        parameters=[{
            'ip_address': "192.168.48.36",
            'port': 7171,
            'arcl_passwd': "omron"
        }]
    )
    ld_states = Node(
        package='om_state_pub',
        executable='om_states_pub',
        #name='amr1_pub',
        #output='screen',
        parameters=[{
            'ip_address': "192.168.250.154",
            'port': 7171,
            'arcl_passwd': "omron",
            'amr_name': "amr1"
        }]
    )

    ld_states2 = Node(
        package='om_state_pub',
        executable='om_states_pub',
        #name='amr1_pub',
        #output='screen',
        parameters=[{
            'ip_address': "192.168.48.36",
            'port': 7171,
            'arcl_passwd': "omron",
            'amr_name': "amr2"
        }]
    )

    return LaunchDescription([
        ld_states,
        #ld_states2,
        action_serve
        #action_serve2
        
    ])



