import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import socketio
sio = socketio.Client()

@sio.event
def connect():
    print('connection established')

@sio.event
def connect_error(data):
    print("The connection failed!")

@sio.event
def disconnect():
    print('disconnected from server')

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'ws_vel', 10)
        self.publisher_hello = self.create_publisher(String, 'hello', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        sio.connect('http://localhost:3003')
        print('my sid is', sio.sid)
        # sio.wait()
        @sio.on('ros:topic')
        def on_message(data):
            # self.get_logger().info()
            d = Twist()
            d.linear.x=float(data['data']['linear'][0])
            d.linear.y=float(data['data']['linear'][1])
            d.linear.z=float(data['data']['linear'][2])
            d.angular.x=float(data['data']['angular'][0])
            d.angular.y=float(data['data']['angular'][1])
            d.angular.z=float(data['data']['angular'][2])
            self.publisher_.publish(d)

    def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_hello.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()