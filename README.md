# prometheus_and_valence_u1

## purpose

1.  Subscribe to the ROS 2 topics: 
    -   */bmu_1/battery_state*.
    -   */bmu_2/battery_state*.
    -   */bmu_3/battery_state*.
2.  Push the *msg.percentage* data from this topic to Prometheus using the *prometheus_client* library's tools on port 9001.
    - The port can be reconfigured during launch using a ROS parameter.

## configure and launch

1.  Configure and start the Prometheus server.
2.  Add this repository to *~/ros2_ws/src*.
3.  Build the workspace.
4.  Source the workspace.
5.  Launch the node with the command:
```
ros2 launch prometheus_and_valence_u1 prometheus_and_valence_u1.py
```
