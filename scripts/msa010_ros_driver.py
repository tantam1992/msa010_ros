#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

import serial
import numpy as np
import cv2
import json


BAUD = {0: 9600, 1: 57600, 2: 115200, 3: 230400, 4: 460800, 5: 921600, 6: 1000000, 7: 2000000, 8: 3000000}


class msa010Driver:
    def __init__(self):
        self.device = rospy.get_param("msa010_ros_driver/device", "/dev/depth_camera")
        self.frame_id = rospy.get_param("msa010_ros_driver/frame_id", "dep_cam_front_link")

        self.depth_img_pub = rospy.Publisher("depth/image_raw", Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher("depth/camera_info", CameraInfo, queue_size=1)

        self.ser = serial.Serial()

        self.ser.port = self.device
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.xonxoff = False
        self.ser.rtscts = False
        self.ser.dsrdtr = False
        self.ser.timeout = 0.2
        # self.ser.write_timeout = 
        # self.ser.inter_byte_timeout = 
        # self.ser.exclusive = 

        self.ser.open()

        print("Connected to Serial: ", self.ser.is_open)
        # self.printSettings()

        self.setSettings(baud_value=5)
        self.printSettings()

        self.fx, self.fy, self.u0, self.v0 = self.intrinsicParam()
        # self.fx, self.fy, self.u0, self.v0 = 75, 75, 50, 50

        self.setSettings(isp_value=1, binn_value=1, unit_value=0, fps_value=10, antimmi_value=-1)
        self.printSettings()

        print("Serial Initialization Completed.")

        self.bridge = CvBridge()
        self.image_array = np.zeros((100, 100), dtype=np.uint8)

        self.header = Header()
        self.header.frame_id = self.frame_id

        self.cam_info = CameraInfo()
        self.cam_info.height = 100
        self.cam_info.width = 100
        self.cam_info.distortion_model = "plumb_bob"
        self.cam_info.D = [0, 0, 0, 0, 0]
        self.cam_info.K = [self.fx, 0, self.u0, 0, self.fy, self.v0, 0, 0, 1]
        self.cam_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        self.cam_info.P = [self.fx, 0, self.u0, 0, 0, self.fy, self.v0, 0, 0, 0, 1, 0]

        self.setSettings(disp_value=4)
        
        print("Start Receiving Depth Image.")

        self.msa010Publisher()


    def closeSerial(self): 
        self.setSettings(disp_value=0)
        print("Serial Closing.")
        self.ser.close()


    def intrinsicParam(self):
        command = "AT+COEFF?\r"
        self.ser.write(command.encode("ASCII"))
        response = self.ser.readlines()
        print("COEFF Response:", response)
        response_string = b''.join(response).decode()
        response_string = response_string.strip()
        response_string = '\n'.join(response_string.split('\n')[2:])
        cparms = json.loads(response_string)
        fx = cparms["fx"] / 262144
        fy = cparms["fy"] / 262144
        u0 = cparms["u0"] / 262144
        v0 = cparms["v0"] / 262144
        print(fx, fy, u0, v0)
        print("------------------------------")

        return fx, fy, u0, v0


    def printSettings(self):
        command = "AT+ISP?\r"
        self.ser.write(command.encode("ASCII"))
        response = self.ser.readlines()
        print("ISP Response:", response)
        command = "AT+BINN?\r"
        self.ser.write(command.encode("ASCII"))
        response = self.ser.readlines()
        print("BINN Response:", response)
        command = "AT+DISP?\r"
        self.ser.write(command.encode("ASCII"))
        response = self.ser.readlines()
        print("DISP Response:", response)
        command = "AT+BAUD?\r"
        self.ser.write(command.encode("ASCII"))
        response = self.ser.readlines()
        print("BAUD Response:", response)
        command = "AT+UNIT?\r"
        self.ser.write(command.encode("ASCII"))
        response = self.ser.readlines()
        print("UNIT Response:", response)
        command = "AT+FPS?\r"
        self.ser.write(command.encode("ASCII"))
        response = self.ser.readlines()
        print("FPS Response:", response)
        command = "AT+ANTIMMI?\r"
        self.ser.write(command.encode("ASCII"))
        response = self.ser.readlines()
        print("ANTIMMI Response:", response)
        print("------------------------------")


    """Set TOF sensor parameters
    Inputs:
        ISP: 0: turn ISP off; 1: turn ISP on
        BINN: 1: output 100x100 pixel frame; 2: output 50x50 pixel frame; 4: output 25x25 pixel frame
        DISP: 0: all off; 1: lcd display on; 2: usb display on; 3: lcd and usb display on; 4: uart display on; 
              5: lcd and uart display on; 6: usb and uart display on; 7: lcd, usb and uart display on
        BAUD: 0: 9600; 1: 57600; 2: 115200; 3: 230400; 4: 460800; 5: 921600; 6: 1000000; 7: 2000000; 8: 3000000
        UNIT: 0: auto; 1-10: quantizated by unit(mm)
        FPS: 1-19: set frame per second
        ANTIMMI: -1: disable anti-mmi; 0: auto anti-mmi; 1-41: manual anti-mmi usb display on

    Return:
        Set parameters through serial
    """
    def setSettings(self, isp_value=None, binn_value=None, disp_value=None, baud_value=None, 
                          unit_value=None, fps_value=None, antimmi_value=None):
        if isp_value is not None: 
            command = "AT+ISP=%1d\r" % isp_value
            self.ser.write(command.encode("ASCII"))
            response = self.ser.readlines()
            print("ISP Response:", response)
        if binn_value is not None: 
            command = "AT+BINN=%1d\r" % binn_value
            self.ser.write(command.encode("ASCII"))
            response = self.ser.readlines()
            print("BINN Response:", response)
        if disp_value is not None: 
            command = "AT+DISP=%1d\r" % disp_value
            self.ser.write(command.encode("ASCII"))
            print("Set DISP value as ", disp_value)
            # response = self.ser.readlines()
            # print("DISP Response:", response)
        if baud_value is not None: 
            command = "AT+BAUD=%1d\r" % baud_value
            self.ser.write(command.encode("ASCII"))
            self.ser.baudrate = BAUD[baud_value]         # change the baudrate of the serial 
            response = self.ser.readlines()
            print("BAUD Response:", response)
        if unit_value is not None: 
            command = "AT+UNIT=%1d\r" % unit_value
            self.ser.write(command.encode("ASCII"))
            response = self.ser.readlines()
            print("UNIT Response:", response)
        if fps_value is not None: 
            command = "AT+FPS=%1d\r" % fps_value
            self.ser.write(command.encode("ASCII"))
            response = self.ser.readlines()
            print("FPS Response:", response)
        if antimmi_value is not None:
            command = "AT+ANTIMMI=%1d\r" % antimmi_value
            self.ser.write(command.encode("ASCII"))
            response = self.ser.readlines()
            print("ANTIMMI Response:", response)
        print("------------------------------")


    def display_image(self, frame_data):
        cv2.imshow("Depth Image", frame_data)
        cv2.waitKey(1)


    def msa010Publisher(self):
        while not rospy.is_shutdown():
            try:
                header = self.ser.read(2)
                # print(header)

                if header == b'\x00\xff':
                    packet_length = self.ser.read(2)

                    other_content = self.ser.read(16)

                    image_data = self.ser.read(10000)

                    check_byte = self.ser.read(1)

                    end = self.ser.read(1)
                    # print(end)

                    if end == b'\xdd':
                        img = np.frombuffer(image_data, dtype=np.uint8)
                        img = np.reshape(img, (100, 100))

                        # header 
                        self.header.stamp = rospy.Time.now()

                        # camera info
                        self.cam_info.header = self.header
                        self.camera_info_pub.publish(self.cam_info)

                        # depth image 
                        img_msg = self.bridge.cv2_to_imgmsg(img, encoding="8UC1")
                        img_msg.header = self.header
                        self.depth_img_pub.publish(img_msg)

                        # image_disp = cv2.resize(img, (500, 500))
                        # self.display_image(image_disp)

                        # print("publishing:", self.header.seq)

            except (serial.SerialException, serial.SerialTimeoutException) as e:
                print(e)
                break

        self.setSettings(disp_value=0)
        print("Serial Closing.")
        self.ser.close()


if __name__ == '__main__':
    try:
        rospy.init_node('msa010_driver', anonymous=True)
        msa010_driver = msa010Driver()
    except rospy.ROSInterruptException:
        msa010_driver.closeSerial()