#!/usr/bin/env python3
"""Save real Stonefish camera frames as an overlapping underwater survey dataset.

Images are saved as PNG alongside timestamps and optional odometry for diagnostics.
The RGB image collection is the input to COLMAP/GLOMAP; no ground-truth images
or poses are given to reconstruction tools. Records are appended to manifest.csv.
"""
import csv
import math
import os
from datetime import datetime
from pathlib import Path
import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from nav_msgs.msg import Odometry

class Capture(Node):
    def __init__(self):
        super().__init__('photogrammetry_capture')
        self.declare_parameter('image_topic','/bluerov2/camera/image_raw/image_color')
        self.declare_parameter('odom_topic','/bluerov2/odometry')
        self.declare_parameter('output_dir',str(Path.home()/'roscon_photogrammetry'/'survey'))
        self.declare_parameter('min_interval_s',1.0)
        self.declare_parameter('min_translation_m',0.18)
        self.declare_parameter('save_static_view_s',6.0)
        self.declare_parameter('max_images',900)
        self.dir=Path(os.path.expanduser(self.get_parameter('output_dir').value))
        (self.dir/'images').mkdir(parents=True,exist_ok=True)
        self.csv_path=self.dir/'manifest.csv'
        if not self.csv_path.exists():
            with self.csv_path.open('w',newline='') as f:
                csv.writer(f).writerow(['image','stamp_sec','stamp_nanosec','odom_x_m','odom_y_m','odom_z_m',
                                         'odom_qx','odom_qy','odom_qz','odom_qw'])
        self.counter=len(list((self.dir/'images').glob('image_*.png')))
        self.last_time=-1e30;self.last_xyz=None;self.odom=None
        self.sub_odom=self.create_subscription(Odometry,self.get_parameter('odom_topic').value,self.on_odom,10)
        self.sub_image=self.create_subscription(ROSImage,self.get_parameter('image_topic').value,self.on_image,10)
        self.get_logger().info('Capturing images to %s (watch for frame count)'%self.dir)

    def on_odom(self,msg):
        self.odom=msg

    @staticmethod
    def decode(msg):
        fmt=msg.encoding.lower()
        formats={'rgb8':(3,'RGB',False),'bgr8':(3,'RGB',True),'rgba8':(4,'RGBA',False),
                 'bgra8':(4,'RGBA',True),'mono8':(1,'L',False)}
        if fmt not in formats:return None
        ch,mode,swap=formats[fmt]
        # honor ROS Image.step (potential row padding)
        expected=int(msg.width)*ch
        if int(msg.step)<expected:return None
        raw=np.frombuffer(bytes(msg.data),dtype=np.uint8)
        if raw.size<int(msg.height)*int(msg.step):return None
        a=raw[:int(msg.height)*int(msg.step)].reshape(int(msg.height),int(msg.step))[:,:expected]
        a=a.reshape(int(msg.height),int(msg.width),ch).copy()
        if swap:
            a=a[:,:,[2,1,0,3]] if ch==4 else a[:,:,::-1]
        return Image.fromarray(a.squeeze(2) if ch==1 else a,mode)

    def on_image(self,msg):
        if self.counter>=int(self.get_parameter('max_images').value):return
        stamp=msg.header.stamp
        t=float(stamp.sec)+float(stamp.nanosec)*1e-9
        if t-self.last_time<max(.1,float(self.get_parameter('min_interval_s').value)):
            return
        pos=None
        if self.odom is not None:
            p=self.odom.pose.pose.position
            pos=(p.x,p.y,p.z)
            if self.last_xyz is not None and math.dist(pos,self.last_xyz)<float(self.get_parameter('min_translation_m').value):
                if t-self.last_time<float(self.get_parameter('save_static_view_s').value):return
        img=self.decode(msg)
        if img is None:
            self.get_logger().warn('Unsupported image encoding %s'%msg.encoding,throttle_duration_sec=5.)
            return
        n=self.counter;self.counter+=1
        fname='image_%05d.png'%n
        img.save(self.dir/'images'/fname)
        if self.odom is not None:
            p=self.odom.pose.pose.position;q=self.odom.pose.pose.orientation
            vals=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        else:vals=['']*7
        with self.csv_path.open('a',newline='') as f:
            csv.writer(f).writerow([fname,stamp.sec,stamp.nanosec]+vals)
        self.last_time=t;self.last_xyz=pos
        if self.counter%10==0:self.get_logger().info('Saved %d survey photos'%self.counter)

def main():
    rclpy.init()
    n=Capture()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
