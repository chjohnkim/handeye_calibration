#!/usr/bin/env python
from scripts.bag_extractor import BagExtractor
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Extract data from bag file')
    parser.add_argument('-d', '--data_root_dir', required=True, type=str, help='path of data root directory')
    parser.add_argument('-b', '--bag_name', required=True, type=str, help='name of bag file without extension')
    parser.add_argument('-s', '--save_every', type=int, default=10, help='down sample extracted data')
    parser.add_argument('-r', '--bag_rate', type=int, default=1, help='bag play speed rate')
    args = parser.parse_args()
    return args




if __name__ == "__main__":
    args = parse_args()
    world_frame = '/world'
    eef_frame = '/wrist_3_link'
    image_topic = '/theia/left/image_raw'
    camera_info_topic = '/theia/left/camera_info'
    be = BagExtractor(args.data_root_dir, args.bag_name, 
                    world_frame, eef_frame, image_topic, camera_info_topic, 
                    save_every=args.save_every, rate=args.bag_rate)

# python extract_bag.py -d data -b harry_lib -s 2 -r 20
