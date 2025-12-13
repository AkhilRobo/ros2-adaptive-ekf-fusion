import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    pkg_name = 'antarctic_robot_description'
    pkg_path = get_package_share_directory(pkg_name)

    urdf_file = os.path.join(pkg_path, 'urdf', 'my_bot.urdf')
    ice_sdf = os.path.join(pkg_path, 'models', 'ice_patch', 'ice_patch.sdf')

    set_prime_offload = SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1')
    set_glx_vendor = SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia')

    robot_desc = ParameterValue(Command(['cat ', urdf_file]), value_type=str)
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        arguments=[]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'antarctic_bot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.5'],
        output='screen'
    )

    spawn_ice = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'ice_patch', '-file', ice_sdf, '-x', '5.0', '-y', '0', '-z', '0.0'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/gps/fix@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_prime_offload,
        set_glx_vendor,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        spawn_ice,
        bridge
    ])