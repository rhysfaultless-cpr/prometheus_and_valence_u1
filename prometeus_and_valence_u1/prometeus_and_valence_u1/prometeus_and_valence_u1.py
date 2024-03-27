#!/usr/bin/env python3
# Software License Agreement (BSD)
#
# @author    Rhys Faultless <rfaultless@clearpathrobotics.com>
# @copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


from prometheus_client import start_http_server, Gauge
prometheus_gauge_1 = Gauge('valence_battery_state_of_charge_1', 'Value of the 1st Valence battery state-of-charge as a percentage')
prometheus_gauge_2 = Gauge('valence_battery_state_of_charge_2', 'Value of the 2nd Valence battery state-of-charge as a percentage')
prometheus_gauge_3 = Gauge('valence_battery_state_of_charge_3', 'Value of the 3rd Valence battery state-of-charge as a percentage')


class PrometheusAndValenceU1(Node):
    def __init__(self):
        super().__init__('PrometheusAndValenceU1')

        self.declare_parameter('port', 9100)
        self.baud = self.get_parameter('port').get_parameter_value().integer_value

        self.battery_percentages = {} # Dictionary to store battery percentages

        self.create_subscription(BatteryState, '/bmu_1/battery_state', self.battery1_state_callback, 10)
        self.create_subscription(BatteryState, '/bmu_2/battery_state', self.battery2_state_callback, 10)
        self.create_subscription(BatteryState, '/bmu_3/battery_state', self.battery3_state_callback, 10)


def battery1_state_callback(self, msg):
    self.battery_percentages[1] = msg.percentage
    prometheus_gauge_1.set(msg.percentage)

def battery2_state_callback(self, msg):
    self.battery_percentages[2] = msg.percentage
    prometheus_gauge_2.set(msg.percentage)

def battery3_state_callback(self, msg):
    self.battery_percentages[3] = msg.percentage
    prometheus_gauge_3.set(msg.percentage)

def main(args=None):
    start_http_server(9100)

    rclpy.init(args=args)
    prometeus_and_valence_u1 = PrometheusAndValenceU1()
    rclpy.spin(prometeus_and_valence_u1)
    prometeus_and_valence_u1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
