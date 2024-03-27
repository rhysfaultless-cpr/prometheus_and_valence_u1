from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  prometeus_and_valence_u1_node = Node(
    package='prometeus_and_valence_u1',
    executable='prometeus_and_valence_u1',
    parameters=[
      {
        'port': 9001
      }
    ]
  )

  ld = LaunchDescription()
  ld.add_action(prometeus_and_valence_u1_node)

  return ld