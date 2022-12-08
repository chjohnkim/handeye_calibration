#!/usr/bin/env python
from scripts.transform_publisher import TransformPublisher
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Extract data from bag file')
    parser.add_argument('-d', '--data_root_dir', required=True, type=str, help='path of data root directory')
    parser.add_argument('-b', '--bag_name', required=True, type=str, help='name of bag file without extension')
    args = parser.parse_args()
    return args


if __name__ == "__main__":
    args = parse_args()
    be = TransformPublisher(args.data_root_dir, args.bag_name)
