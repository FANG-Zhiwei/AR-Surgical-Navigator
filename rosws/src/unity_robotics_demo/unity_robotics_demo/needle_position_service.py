#!/usr/bin/env python

import random
import rclpy

from unity_robotics_demo_msgs.srv import NeedlePosService

from rclpy.node import Node

class NeedlePosServiceNode(Node):

    def __init__(self):
        super().__init__('needle_position_service_node')
        self.srv = self.create_service(NeedlePosService, 'needle_pos_srv', self.new_position_callback)

    def new_position_callback(self, request, response):
        response.output.pos_x = random.uniform(-4.0, 4.0)
        response.output.pos_z = random.uniform(-4.0, 4.0)
        
        return response


def main(args=None):
    rclpy.init(args=args)

    pos_service = NeedlePositionServiceNode()

    rclpy.spin(pos_service)
	
    rclpy.shutdown()


if __name__ == '__main__':
    main()
