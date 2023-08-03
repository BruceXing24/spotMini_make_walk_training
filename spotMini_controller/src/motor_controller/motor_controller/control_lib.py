#!/usr/bin/env python3
import time
import rclpy
import serial
import struct
import math
import numpy as np

serialHandle = serial.Serial("/dev/ttyAMA0", 9600)


class MotorControl:
    def __init__(self):
        self.motor_num = 12
        self.hip_limit = [-30,  30]
        self.uLed_limit = [ -90, 90]
        self.lLed_limit = [-91, 91]
        self.motor_wise = [1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1]
        self.res = 1000. / 240.
        # self.control_fre = 1. / 50 * 1000  # unit ms
        self.control_fre = 500  # unit ms


        self.angle_error = [0,10,25, 20,0,0, -20,-20,25, 30,-25,0]


    def check_angle(self, motor_angles: list):

        assert len(motor_angles) == 12, 'motors angle list length wrong'
        for i in range(4):
            motor_angles[i * 3] =     np.clip(motor_angles[i * 3],     self.hip_limit[0],  self.hip_limit[1])
            motor_angles[i * 3 + 1] = np.clip(motor_angles[i * 3 + 1], self.uLed_limit[0], self.uLed_limit[1])
            motor_angles[i * 3 + 2] = np.clip(motor_angles[i * 3 + 2], self.lLed_limit[0], self.lLed_limit[1])
        return motor_angles

    def angle_to_command(self, motor_angles: list):
        command_valus = [0,0,0, 0,0,0, 0,0,0, 0,0,0]
        # make sure all 12 motors use the same coordinate as whole body.
        # FL:1  2  3  FR:4  5  6  HL:7  8  9  HR:10  11  12
        #    +  -  -     -  +  +     +  -  -     -    +   +
        assert len(motor_angles) == 12, 'motors angle list length wrong'
        for i in range(12):
            if self.motor_wise[i] == 0:
                command_valus[i] = self.res * motor_angles[i]+500
            else:
                command_valus[i] = -self.res * motor_angles[i]+500
            print(f"selfres=={self.res}delta===={self.res * motor_angles[i]}")
        return command_valus

    def command_on_motor(self, num, time, command_values: list):
        id = [1,2,3,4,5,6,7,8,9,10,11,12]
        print(command_values)
        i = 0
        buf = [0x55, 0x55]
        length = 3 * num + 5
        cmd = 0x03
        buf.extend([0xff & length, 0xff & cmd, 0xff & num])
        time_h = bytearray(struct.pack('h', int(time)))
        buf.append(time_h[0])
        buf.append(time_h[1])
        while (i < num):
            command_values[i] = np.int16(command_values[i])
            # print(type(command_values[i]))
            buf.extend([0xff & id[i], 0xff&command_values[i], 0xff& command_values[i]>>8])
            i += 1
        serialHandle.write(buf)

    def control_motors(self, motor_angles: list):
        assert len(motor_angles) == 12, 'motors angle list length wrong'
        motor_angles = self.check_angle(motor_angles)
        command_values = self.angle_to_command(motor_angles)
        cali_values = self.cali_values(command_values)
        self.command_on_motor(12, self.control_fre, cali_values)

    def cali_values(self, command_values):
        for i in range(12):
            command_values[i] = command_values[i]+self.angle_error[i]
        return command_values

    def test_single_leg(self, id, time, command):
        command = np.clip(command,300,700)
        buf = [0x55, 0x55]
        length = 3 * 1 + 5
        cmd = 0x03
        buf.extend([0xff & length, 0xff & cmd, 0xff & 1])
        time_h = bytearray(struct.pack('h', int(time)))
        buf.append(time_h[0])
        buf.append(time_h[1])
        buf.extend([0xff & id, 0xff & command, 0xff & command >> 8])
        serialHandle.write(buf)

if __name__ == '__main__':
    controller = MotorControl()
    #stand_angle = [0,45,-90, 0,45,-90, 0,45,-90, 0,45,-90]
    initial_angle = [0,0,0, 0,0,0, 0,0,0, 0,0,0]
    test_angle = [15,0,0, 15,0,0, 15,0,0, 15,0,0]
    controller.control_motors(initial_angle)
    time.sleep(5)
    controller.control_motors(test_angle)
