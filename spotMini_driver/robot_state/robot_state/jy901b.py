#!/usr/bin/env python3
import serial
import time
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import rclpy

class IMU(Node):
    def __init__(self,name):
        super(IMU,self).__init__(name)
        # Initial conditions
        self.ser = serial.Serial()
        self.ser.port = '/dev/ttyUSB0' #for linux. Also change the USB# to the correct # if necessary.
        #ser.port = 'COM7' 
        self.ser.baudrate = 115200
        self.ser.parity = 'N'
        self.ser.bytesize = 8
        self.ser.timeout = 1
        self.ser.open()
        self.readData = ""
        self.dataStartRecording = False

        self.rpyPulisher = self.create_publisher(Vector3,'/rpy_angle',1)
        self.WxyzPulisher = self.create_publisher(Vector3,'/Wxyz',1)

        self.rpy = Vector3()
        self.Wxyz = Vector3()
        # This part is needed so that the reading can start reliably.
        print('Starting...', self.ser.name)
        time.sleep(1)
        self.ser.reset_input_buffer()
	  
        # self.create_timer(0.,callback=self.timer_callback)
        self.create_timer(0.02,callback=self.pub_timer_callback)

    # def timer_callback(self):
    #     rawData = self.ser.read(size=2).hex()

    # # Make sure that ALL self.readData starts with 5551 before recording starts. Readings do not always start at bytes 5551 for some reason.	
    #     if rawData == '5551':
    #         self.dataStartRecording = True
        
    # # Recording and concatenation functions
    #     if self.dataStartRecording == True:
    #         self.readData = self.readData + rawData
            
    # # Processing. Varaible names based on variables on the Witmotion WT901CTTL datasheet.
    #         if len(self.readData) == 88:
    #             StartAddress_1 = int(self.readData[0:2], 16)
    #             StartAddress_A = int(self.readData[2:4], 16)
    #             AxL = int(self.readData[4:6], 16)
    #             AxH = int(self.readData[6:8], 16)
    #             AyL = int(self.readData[8:10], 16)
    #             AyH = int(self.readData[10:12], 16)
    #             AzL = int(self.readData[12:14], 16)
    #             AzH = int(self.readData[14:16], 16)
    #             TL_A = int(self.readData[16:18], 16)
    #             TH_A = int(self.readData[18:20], 16)
    #             SUM_A = int(self.readData[20:22], 16)
                
    #             StartAddress_2 = int(self.readData[22:24], 16)
    #             StartAddress_w = int(self.readData[24:26], 16)
    #             wxL = int(self.readData[26:28], 16)
    #             wxH = int(self.readData[28:30], 16)
    #             wyL = int(self.readData[30:32], 16)
    #             wyH = int(self.readData[32:34], 16)
    #             wzL = int(self.readData[34:36], 16)
    #             wzH = int(self.readData[36:38], 16)
    #             TL_w = int(self.readData[38:40], 16)
    #             TH_w = int(self.readData[40:42], 16)
    #             SUM_w = int(self.readData[42:44], 16)
                
    #             StartAddress_3 = int(self.readData[44:46], 16)
    #             StartAddress_ypr = int(self.readData[46:48], 16)
    #             RollL = int(self.readData[48:50], 16)
    #             RollH = int(self.readData[50:52], 16)
    #             PitchL = int(self.readData[52:54], 16)
    #             PitchH = int(self.readData[54:56], 16)
    #             YawL = int(self.readData[56:58], 16)
    #             YawH = int(self.readData[58:60], 16)
    #             VL = int(self.readData[60:62], 16)
    #             VH = int(self.readData[62:64], 16)
    #             SUM_ypr = int(self.readData[64:66], 16)
                
    #             StartAddress_4 = int(self.readData[66:68], 16)
    #             StartAddress_mag = int(self.readData[68:70], 16)
    #             HxL = int(self.readData[70:72], 16)
    #             HxH = int(self.readData[72:74], 16)
    #             HyL = int(self.readData[74:76], 16)
    #             HyH = int(self.readData[76:78], 16)
    #             HzL = int(self.readData[78:80], 16)
    #             HzH = int(self.readData[80:82], 16)
    #             TL_mag = int(self.readData[82:84], 16)
    #             TH_mag = int(self.readData[84:86], 16)
    #             SUM_mag = int(self.readData[86:88], 16)
                
    # # Acceleration output
    #             Ax = float(np.short((AxH<<8)|AxL)/32768.0*16.0)
    #             Ay = float(np.short((AyH<<8)|AyL)/32768.0*16.0)
    #             Az = float(np.short((AzH<<8)|AzL)/32768.0*16.0)
    #             T_A = float(np.short((TH_A<<8)|TL_A)/100.0)
                
    # # Angular velocity output		
    #             Wx = float(np.short((wxH<<8)|wxL)/32768.0*2000.0)
    #             Wy = float(np.short((wyH<<8)|wyL)/32768.0*2000.0)
    #             Wz = float(np.short((wzH<<8)|wzL)/32768.0*2000.0)
    #             T_w = float(np.short((TH_w<<8)|TL_w) /100.0)
                
    # # Angle output			
    #             Roll = float(np.short((RollH<<8)|RollL)/32768.0*180.0)
    #             Pitch = float(np.short((PitchH<<8)|PitchL)/32768.0*180.0)
    #             Yaw = float(np.short((YawH<<8)|YawL)/32768.0*180.0)
                
    # # Magnetic output
    #             Hx = float(np.short(HxH<<8)| HxL)
    #             Hy = float(np.short(HyH<<8)| HyL)
    #             Hz = float(np.short(HzH<<8)| HzL)
    #             T_mag = float(np.short((TH_mag<<8)|TL_mag) /100.0)

    # # Readable outputs. Uncomment for specific readouts. 
    #             #print(self.readData)
    #             #print("%6.3f" % Ax, "%6.3f" % Ay, "%6.3f" % Az)
    #             #print("%7.3f" % Wx, "%7.3f" % Wy, "%7.3f" % Wz) # This detects any movement on the axes.
    #             print("%7.3f" % Roll, "%7.3f" % Pitch, "%7.3f" % Yaw) # This maps out tilt angles of the axes.
    #             #print("%4.0f" % Hx, "%4.0f" % Hy, "%4.0f" % Hz)
    # # Cleanup
    #             self.readData = ""
    #             self.dataStartRecording = False
                
    #             self.rpy.x = Roll
    #             self.rpy.y = Pitch
    #             self.rpy.z = Yaw
                
    #             self.Wxyz.x = Wx
    #             self.Wxyz.y = Wy
    #             self.Wxyz.z = Wz

    def pub_timer_callback(self):
        self.rpyPulisher.publish(self.rpy)
        self.WxyzPulisher.publish(self.Wxyz)


    def read_data(self):
        while True:
            rawData = self.ser.read(size=2).hex()

        # Make sure that ALL self.readData starts with 5551 before recording starts. Readings do not always start at bytes 5551 for some reason.	
            if rawData == '5551':
                self.dataStartRecording = True
            
        # Recording and concatenation functions
            if self.dataStartRecording == True:
                self.readData = self.readData + rawData
                
        # Processing. Varaible names based on variables on the Witmotion WT901CTTL datasheet.
                if len(self.readData) == 88:
                    StartAddress_1 = int(self.readData[0:2], 16)
                    StartAddress_A = int(self.readData[2:4], 16)
                    AxL = int(self.readData[4:6], 16)
                    AxH = int(self.readData[6:8], 16)
                    AyL = int(self.readData[8:10], 16)
                    AyH = int(self.readData[10:12], 16)
                    AzL = int(self.readData[12:14], 16)
                    AzH = int(self.readData[14:16], 16)
                    TL_A = int(self.readData[16:18], 16)
                    TH_A = int(self.readData[18:20], 16)
                    SUM_A = int(self.readData[20:22], 16)
                    
                    StartAddress_2 = int(self.readData[22:24], 16)
                    StartAddress_w = int(self.readData[24:26], 16)
                    wxL = int(self.readData[26:28], 16)
                    wxH = int(self.readData[28:30], 16)
                    wyL = int(self.readData[30:32], 16)
                    wyH = int(self.readData[32:34], 16)
                    wzL = int(self.readData[34:36], 16)
                    wzH = int(self.readData[36:38], 16)
                    TL_w = int(self.readData[38:40], 16)
                    TH_w = int(self.readData[40:42], 16)
                    SUM_w = int(self.readData[42:44], 16)
                    
                    StartAddress_3 = int(self.readData[44:46], 16)
                    StartAddress_ypr = int(self.readData[46:48], 16)
                    RollL = int(self.readData[48:50], 16)
                    RollH = int(self.readData[50:52], 16)
                    PitchL = int(self.readData[52:54], 16)
                    PitchH = int(self.readData[54:56], 16)
                    YawL = int(self.readData[56:58], 16)
                    YawH = int(self.readData[58:60], 16)
                    VL = int(self.readData[60:62], 16)
                    VH = int(self.readData[62:64], 16)
                    SUM_ypr = int(self.readData[64:66], 16)
                    
                    StartAddress_4 = int(self.readData[66:68], 16)
                    StartAddress_mag = int(self.readData[68:70], 16)
                    HxL = int(self.readData[70:72], 16)
                    HxH = int(self.readData[72:74], 16)
                    HyL = int(self.readData[74:76], 16)
                    HyH = int(self.readData[76:78], 16)
                    HzL = int(self.readData[78:80], 16)
                    HzH = int(self.readData[80:82], 16)
                    TL_mag = int(self.readData[82:84], 16)
                    TH_mag = int(self.readData[84:86], 16)
                    SUM_mag = int(self.readData[86:88], 16)
                    
        # Acceleration output
                    Ax = float(np.short((AxH<<8)|AxL)/32768.0*16.0)
                    Ay = float(np.short((AyH<<8)|AyL)/32768.0*16.0)
                    Az = float(np.short((AzH<<8)|AzL)/32768.0*16.0)
                    T_A = float(np.short((TH_A<<8)|TL_A)/100.0)
                    
        # Angular velocity output		
                    Wx = float(np.short((wxH<<8)|wxL)/32768.0*2000.0)
                    Wy = float(np.short((wyH<<8)|wyL)/32768.0*2000.0)
                    Wz = float(np.short((wzH<<8)|wzL)/32768.0*2000.0)
                    T_w = float(np.short((TH_w<<8)|TL_w) /100.0)
                    
        # Angle output			
                    Roll = float(np.short((RollH<<8)|RollL)/32768.0*180.0)
                    Pitch = float(np.short((PitchH<<8)|PitchL)/32768.0*180.0)
                    Yaw = float(np.short((YawH<<8)|YawL)/32768.0*180.0)
                    
        # Magnetic output
                    Hx = float(np.short(HxH<<8)| HxL)
                    Hy = float(np.short(HyH<<8)| HyL)
                    Hz = float(np.short(HzH<<8)| HzL)
                    T_mag = float(np.short((TH_mag<<8)|TL_mag) /100.0)

        # Readable outputs. Uncomment for specific readouts. 
                    #print(self.readData)
                    #print("%6.3f" % Ax, "%6.3f" % Ay, "%6.3f" % Az)
                    #print("%7.3f" % Wx, "%7.3f" % Wy, "%7.3f" % Wz) # This detects any movement on the axes.
                    #print("%7.3f" % Roll, "%7.3f" % Pitch, "%7.3f" % Yaw) # This maps out tilt angles of the axes.
                    #print("%4.0f" % Hx, "%4.0f" % Hy, "%4.0f" % Hz)
        # Cleanup
                    self.readData = ""
                    self.dataStartRecording = False
                    
                    self.rpy.x = Roll
                    self.rpy.y = Pitch
                    self.rpy.z = Yaw
                    
                    self.Wxyz.x = Wx
                    self.Wxyz.y = Wy
                    self.Wxyz.z = Wz
                    self.rpyPulisher.publish(self.rpy)
                    self.WxyzPulisher.publish(self.Wxyz)  

def main():
    rclpy.init()
    node = IMU('imu_jy901b')
    node.read_data()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()  
