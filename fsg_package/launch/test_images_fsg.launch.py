from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="fsg_package",
            executable="fsg",
            name="fsg",
#            remappings=[
#              ('/output_image', '/input_image')
#            ]
        ),
        Node(
            package="fsg_package",
            executable="test_input",
            name="test_input",
            parameters=[
                {"resize_factor": 0.2},
                {"video_location": "testvid.mp4"},
            ],
            output="screen",
        ),
    ])
