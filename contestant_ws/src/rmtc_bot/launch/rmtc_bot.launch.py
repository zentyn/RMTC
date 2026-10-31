from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    team = LaunchConfiguration('team')
    name = LaunchConfiguration('name')
    gateway = LaunchConfiguration('gateway')
    token = LaunchConfiguration('token')

    return LaunchDescription([
        DeclareLaunchArgument('team', default_value='A', description='本场席位 A/B/C/D'),
        DeclareLaunchArgument('name', default_value='我的队伍', description='注册显示名称'),
        DeclareLaunchArgument('gateway', default_value='http://127.0.0.1:8080',
                              description='统一 AI 网关地址'),
        DeclareLaunchArgument('token', default_value='practice-A', description='席位令牌'),
        Node(
            package='rmtc_bot',
            executable='rmtc_bot_node',
            name='rmtc_bot',
            output='screen',
            arguments=[
                '--team', team,
                '--name', name,
                '--gateway', gateway,
                '--token', token,
            ],
        ),
    ])
