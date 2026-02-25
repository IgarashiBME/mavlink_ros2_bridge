from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Launch arguments: existing parameters ---
    gcs_ip_arg = DeclareLaunchArgument(
        'gcs_ip', default_value='127.0.0.1',
        description='IP address of the GCS (QGroundControl)')

    param_file_path_arg = DeclareLaunchArgument(
        'param_file_path',
        default_value='~/.ros/mavlink_bridge_params.yaml',
        description='Path to parameter persistence file')

    # --- Launch arguments: topic remapping ---
    # Subscribers
    gnss_topic_arg = DeclareLaunchArgument(
        'gnss_topic', default_value='/gnss/solution',
        description='GNSS input topic (bme_common_msgs/GnssSolution)')

    auto_log_topic_arg = DeclareLaunchArgument(
        'auto_log_topic', default_value='/auto_log',
        description='Auto log input topic (std_msgs/Float64MultiArray)')

    # Publishers
    mission_topic_arg = DeclareLaunchArgument(
        'mission_topic', default_value='/mav/mission',
        description='Mission output topic (std_msgs/Float64MultiArray)')

    modes_topic_arg = DeclareLaunchArgument(
        'modes_topic', default_value='/mav/modes',
        description='Modes output topic (bme_common_msgs/MavModes)')

    joystick_topic_arg = DeclareLaunchArgument(
        'joystick_topic', default_value='/mav/joystick',
        description='Joystick output topic (geometry_msgs/TwistStamped)')

    mission_set_current_topic_arg = DeclareLaunchArgument(
        'mission_set_current_topic', default_value='/mav/mission_set_current',
        description='Mission set current output topic (std_msgs/UInt16)')

    # --- Node ---
    mavlink_bridge_node = Node(
        package='mavlink_ros2_bridge',
        executable='mavlink_bridge_node',
        name='mavlink_bridge_node',
        output='screen',
        parameters=[{
            'gcs_ip': LaunchConfiguration('gcs_ip'),
            'param_file_path': LaunchConfiguration('param_file_path'),
}],
        remappings=[
            ('/gnss', LaunchConfiguration('gnss_topic')),
            ('/auto_log', LaunchConfiguration('auto_log_topic')),
            ('/mav/mission', LaunchConfiguration('mission_topic')),
            ('/mav/modes', LaunchConfiguration('modes_topic')),
            ('/mav/joystick', LaunchConfiguration('joystick_topic')),
            ('/mav/mission_set_current',
             LaunchConfiguration('mission_set_current_topic')),
        ],
    )

    return LaunchDescription([
        # Parameters
        gcs_ip_arg,
        param_file_path_arg,
        # Topic remapping
        gnss_topic_arg,
        auto_log_topic_arg,
        mission_topic_arg,
        modes_topic_arg,
        joystick_topic_arg,
        mission_set_current_topic_arg,
        # Node
        mavlink_bridge_node,
    ])
