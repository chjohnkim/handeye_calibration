#!/usr/bin/env python
from scripts.pose_pair_saver import PosePairSaver
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Extract data from bag file')
    parser.add_argument('-d', '--data_root_dir', required=True, type=str, help='path of data root directory')
    parser.add_argument('-b', '--bag_name', required=True, type=str, help='name of bag file without extension')
    args = parser.parse_args()
    return args


if __name__ == "__main__":
    args = parse_args()
    be = PosePairSaver(args.data_root_dir, args.bag_name)
# python extract_bag.py -d data -b harry_lib
