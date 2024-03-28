# prometheus_and_valence_u1

## purpose

This package subscribes to ROS 2 topics, and then adds the data to a Prometheus database.
This is then pulled into Grafana.

<p align="center"><img src="/readme_assets/grafana_dashboard.png" width="800"/></p>

<br />

## tasks

1.  Subscribe to the ROS 2 topics: 
    -   */bmu_1/battery_state*.
    -   */bmu_2/battery_state*.
    -   */bmu_3/battery_state*.
    -   */numato_relay_state_0*.
2.  Push the data from these topics to Prometheus using the *prometheus_client* library's tools on port 9100.

## configure and launch

1.  Configure and start the Prometheus server.
2.  Add this repository to *~/ros2_ws/src*.
3.  Build the workspace.
4.  Source the workspace.
5.  Launch the node with the command:
```
ros2 launch prometheus_and_valence_u1 prometheus_and_valence_u1.py
```
