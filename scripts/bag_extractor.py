##!/usr/bin/env python
import subprocess
import os
import signal
import yaml

import rospy
import message_filters
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import cv2
import tf
import argparse

class BagExtractor():
    def __init__(self, data_root_dir, bag_dir, world_frame, eef_frame, image_topic, camera_info_topic, save_every=1, rate=1):
        
        rospy.init_node('bag_extractor', anonymous=True)
        self.world_frame  = world_frame
        self.eef_frame = eef_frame
        self.image_topic = image_topic
        self.camera_info_topic = camera_info_topic

        self.data_root_dir = data_root_dir
        self.bag_dir = bag_dir
        bag_full_path = os.path.join(data_root_dir, bag_dir, bag_dir+'.bag')
        self.command = 'rosbag play -r {} {}'.format(rate, bag_full_path)
        
        self.bag_info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bag_full_path], stdout=subprocess.PIPE).communicate()[0])
        print(yaml.safe_dump(self.bag_info_dict, default_flow_style=False))
        bag_info_path = os.path.join(self.data_root_dir, bag_dir, 'bag_info.yaml')
        with open(bag_info_path, 'w+') as f:    
            yaml.safe_dump(self.bag_info_dict, f, default_flow_style=False)
        self.count = 0        
        self.save_every = save_every
        self.pose_data = dict()
        self.create_directories()    
        self.play_bag()
        self.save_data()
        self.close()
    
    def create_directories(self):
        directories = []
        directories.append(os.path.join(self.data_root_dir, self.bag_dir, 'images'))
        for dir in directories:        
            if not os.path.exists(dir):
                os.makedirs(dir)

    def play_bag(self):
        self.p = subprocess.Popen(self.command, stdin=subprocess.PIPE, shell=True, executable='/bin/bash')

    def save_data(self):
        # save camera info
        camera_info_msg = rospy.wait_for_message(self.camera_info_topic, CameraInfo, timeout=5)
        camera_info_dict = self.parse_camera_info_msg(camera_info_msg)
        camera_info_path = os.path.join(self.data_root_dir, self.bag_dir, 'camera_info.yaml')
        with open(camera_info_path, 'w+') as f:
            yaml.safe_dump(camera_info_dict, f, default_flow_style=False)
        
        # subscribers
        tf_listener = tf.TransformListener()
        subscriber_rgb = message_filters.Subscriber(self.image_topic, Image)
 
        bridge = CvBridge()
        image_save_path = os.path.join(self.data_root_dir, self.bag_dir, 'images')        
        rospy.sleep(1)
 
        def callback(rgb):
            if self.count%self.save_every == 0:
                idx = int(self.count/self.save_every)
                (trans,rot) = tf_listener.lookupTransform(self.world_frame, self.eef_frame, rospy.Time(0))

                img_rgb_np = bridge.imgmsg_to_cv2(rgb, desired_encoding='passthrough')
                img_rgb_np = cv2.cvtColor(img_rgb_np, cv2.COLOR_RGB2BGR)

                cv2.imwrite(os.path.join(image_save_path, '{:06d}_rgb.png'.format(idx)), img_rgb_np)
                
                self.pose_data.update(
                    {idx: {
                            'quaternion': {
                                'w': float(rot[3]),
                                'x': float(rot[0]),
                                'y': float(rot[1]),
                                'z': float(rot[2])
                            },
                            'translation': {
                                'x': float(trans[0]),
                                'y': float(trans[1]),
                                'z': float(trans[2])
                            },
                        'rgb_image_filename': '{:06d}_rgb.png'.format(idx),
                        'timestamp': rgb.header.stamp.to_sec()
                        }
                    }
                )
            self.count+=1

        ts = message_filters.ApproximateTimeSynchronizer([subscriber_rgb], 10, 0.1, allow_headerless=True)
        ts.registerCallback(callback)
        rospy.sleep(self.bag_info_dict['duration'])
        
        pose_data_path = os.path.join(self.data_root_dir, self.bag_dir, 'pose_data.yaml')
        with open(pose_data_path, 'w+') as f:    
            yaml.safe_dump(self.pose_data, f, default_flow_style=False)
        #rospy.spin()

    def parse_camera_info_msg(self, msg):
        camera_info_dict = dict()
        camera_info_dict.update(
            {
                'camera_matrix': {
                    'cols': 3,
                    'data': msg.K,
                    'rows': 3
                },
                'distortion_coefficients': {
                    'cols': 5,
                    'data': msg.D,
                    'rows': 1
                },
                'distortion_model': msg.distortion_model,
                'image_height': msg.height,
                'image_width': msg.width,
                'projection_matrix': {
                    'cols': 4,
                    'data': msg.P,
                    'rows': 3
                },
                'rectification_matrix': {
                    'cols': 3,
                    'data': msg.R,
                    'rows': 3
                },
            }
        )
        return camera_info_dict

    def close(self):
        import psutil
        process = psutil.Process(self.p.pid)
        for sub_process in process.children(recursive=True):
            sub_process.send_signal(signal.SIGINT)
        self.p.wait()  # we wait for children to terminate
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract data from bag file')
    parser.add_argument('-d', '--data_root_dir', required=True, type=str, help='path of data root directory')
    parser.add_argument('-b', '--bag_name', required=True, type=str, help='name of bag file without extension')
    parser.add_argument('-s', '--save_every', type=int, default=10, help='down sample extracted data')
    parser.add_argument('-r', '--bag_rate', type=int, default=1, help='bag play speed rate')
    args = parser.parse_args()

    world_frame = '/world'
    eef_frame = '/wrist_3_link'
    image_topic = '/theia/left/image_raw'
    camera_info_topic = '/theia/left/camera_info'
    be = BagExtractor(args.data_root_dir, args.bag_name, 
                    world_frame, eef_frame, image_topic, camera_info_topic, 
                    save_every=args.save_every, rate=args.bag_rate)