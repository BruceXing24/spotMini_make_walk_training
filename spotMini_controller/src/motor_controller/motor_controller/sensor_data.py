import rclpy
import serial
import struct
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
import numpy as np
import time

PI = 3.1415926
class SensorData(Node):
    def __init__(self,name):
        super(SensorData, self).__init__(name)
        self.ser = serial.Serial("/dev/ttyUSB0",115200)
        self.rpyPulisher = self.create_publisher(Vector3,'/rpy_angle',1)
        self.disPublisher = self.create_publisher(Int32,'/height',1)
        self.angAccPublisher = self.create_publisher(Vector3,'/angularAccel',1)
        self.create_timer(0.01,self.timer_callback)
        self.rpy = Vector3()
        self.angularAccel = Vector3()
        self.distance = Int32()

    def timer_callback(self):
        data = self.ser.read(25)
        distance = struct.unpack('<h',data[2:4])
        roll = struct.unpack('f',data[4:8])
        pitch = struct.unpack('f',data[8:12])
        wx = struct.unpack('f',data[12:16])
        wy = struct.unpack('f',data[16:20])
        wz = struct.unpack('f',data[20:24])

        self.distance.data = int(distance[0])
        self.rpy.x = float(roll[0])*180/PI
        self.rpy.y = float(pitch[0])*180/PI
        self.rpy.z = 0.

        if np.abs(int(distance[0])) > 1000 or np.abs(float(roll[0])*180/PI)>1000:
            self.get_logger().info("imu receive wrong, reseting")
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            time.sleep(2)
            self.get_logger().info("reset done")



        self.angularAccel.x = float(wx[0])
        self.angularAccel.y = float(wy[0])
        self.angularAccel.z = float(wz[0])
        self.disPublisher.publish(self.distance)
        self.rpyPulisher.publish(self.rpy)



def main():
    rclpy.init()
    sensor_node = SensorData('sensor_data')
    rclpy.spin(sensor_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
