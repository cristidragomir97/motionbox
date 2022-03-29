#! /usr/bin/env python
import time, rospy, queue, threading, TCA9548A
from MPU6050 import MPU6050

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, TransformStamped, Transform, Quaternion
from tf2_ros import TransformBroadcaster
from tf.transformations import *

BUF_SIZE = 25
q = queue.Queue(BUF_SIZE)

class Sensor(object):
    def __init__(self):
        self.offsets = {
            'x_accel_offset' : int(-841),
            'y_accel_offset' : int(1794),
            'z_accel_offset' : int(3206),
            'x_gyro_offset' : int(26),
            'y_gyro_offset' : int(28),
            'z_gyro_offset' : int(26),
        }

        mpu = MPU6050(1, 0x68,
            self.offsets['x_accel_offset'],
            self.offsets['y_accel_offset'],
            self.offsets['z_accel_offset'],
            self.offsets['x_gyro_offset'],
            self.offsets['y_gyro_offset'],
            self.offsets['z_gyro_offset'],
            False)
        
        self.mpu = mpu 
        self.mpu.reset_FIFO()
        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)
        self.mpu_int_status = mpu.get_int_status()
        self.packet_size = mpu.DMP_get_FIFO_packet_size()
        self.FIFO_count = mpu.get_FIFO_count()

        self.FIFO_buffer = [0]*64

        print(self.mpu_int_status, self.packet_size, self.FIFO_count)

    def read(self):
        euler = []
        try:
            self.FIFO_count = self.mpu.get_FIFO_count()
            self.mpu_int_status = self.mpu.get_int_status()
        except:
            pass
            
        # If overflow is detected by status or fifo count we want to reset
        if (self.FIFO_count == 1024) or (self.mpu_int_status & 0x10):
            self.mpu.reset_FIFO()
            return None
            
        # Check if fifo data is ready
        elif (self.mpu_int_status & 0x02):

            # Wait until packet_size number of bytes are ready for reading, default is 42 bytes
            while self.FIFO_count < self.packet_size:
                self.FIFO_count = self.mpu.get_FIFO_count()
            
            self.FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)

            quat = self.mpu.DMP_get_quaternion(self.FIFO_buffer)
            return quat

class SensorListener(threading.Thread):
    def __init__(self, segments):
        super(SensorListener,self).__init__()
        self.segments = segments 
        self.sensors = {}
        for segment in segments:
            TCA9548A.switch(segment)
            self.sensors[segment] = Sensor()
            time.sleep(0.25)
    
    def run(self):
        while True:
            for segment in self.segments:
                TCA9548A.switch(segment)
                try:
                    if not q.full():
                        quaternion = self.sensors[segment].read()
                        if quaternion is not None:
                            obj = [quaternion, self.segments[segment]]
                            q.put(obj)
                except Exception as e: 
                    print(e)

class SensorBroadcaster(threading.Thread):
    def __init__(self):
        super(SensorBroadcaster,self).__init__()
        self.broadcaster = TransformBroadcaster()
        rospy.init_node('wrist_tf', anonymous=False)
        rate = rospy.Rate(60)

    def send_tf(self, origin, q, name, base_frame):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = base_frame
        t.child_frame_id = name
        t.transform.translation.x  = origin[0]
        t.transform.translation.y  = origin[1] 
        t.transform.translation.z  = origin[2]
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
        self.broadcaster.sendTransform(t)

    def run(self):
        while True:
            if not q.empty():
                quat, segment = q.get()
                seg = segment["segment"]
                base_frame = segment["base_frame"]
                origin = segment["origin"]
                self.send_tf(origin, quat, seg, base_frame)

        return 

if __name__ == "__main__":
    broadcaster = SensorBroadcaster()
    listener = SensorListener({
        2 : {'segment': 'upper_arm', 'base_frame': 'base_link', 'origin': [0.0, 0.5, 0.0]},
        3 : {'segment': 'lower_arm', 'base_frame': 'upper_arm', 'origin': [0.0, 0.5, 0.0]},
        6 : {'segment': 'hand', 'base_frame': 'lower_arm', 'origin': [0.0, 0.5, 0.0]},
    })

    listener.start()
    time.sleep(2)
    broadcaster.start()