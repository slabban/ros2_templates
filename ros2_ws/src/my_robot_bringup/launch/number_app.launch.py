from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # The node constructor will accept the typical variables that we used to 
    # pass into the ros1:
    # package: package name
    # executable: name of executable file
    # name: arbitrary name give to node
    # remappings: remapping topic names, this MUST be an array of tuples
    # paramters: set parameters, the MUST be an array of dictionaries
    number_publisher_node = Node(
        package="ros2_app_cpp",
        executable="number_publisher",
        name="my_number_publisher",
        remappings=[
            ("number", "my_number")
        ]
    )

    number_counter_node = Node(
        package="ros2_app_cpp",
        executable="number_counter",
        name="my_number_counter",
        remappings=[
            ("number", "my_number")
        ]
    )

    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)

    return ld