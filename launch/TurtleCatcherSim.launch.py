from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   
    TurtleSimCatcher = Node(
        package='TurtleCatcherSim',
        executable='TurtleCatcherSim.py',
        name='TurtleCatcherSim'
    )

    TurtleSimNode = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node'
    )


    launch_description = LaunchDescription()
    launch_description.add_action(TurtleSimNode)
    launch_description.add_action(TurtleSimCatcher)

    return launch_description