from launch import LaunchDescription
from launch_ros.actions import Node
import re


def generate_launch_description():
    ld = LaunchDescription()

    robot_names = ["R2D2", "C3PO", "T-1000",
                   "Johnny 5", "David 8"]

    robot_news_station_nodes = []

    pattern = re.compile("\W")

    for name in robot_names:
        node_name = re.sub("\W", "_", name)
        robot_news_station_nodes.append(
            Node(
                package="my_cpp_pkg",
                executable="robot_news_station",
                name="robot_news_station_" + node_name.lower(),
                parameters=[
                    {"robot_name": name},
                ]
            )
        )

    smartphone_node = Node(
        package="my_cpp_pkg",
        executable="smartphone",
        name="my_smartphone",
    )

    for node in robot_news_station_nodes:
        ld.add_action(node)

    ld.add_action(smartphone_node)

    return ld
