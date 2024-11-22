##!/usr/bin/env python
import os
import scipy.io
import numpy as np
from scipy.spatial.transform import Rotation as R
import argparse

class PosePairSaver():
    def __init__(self, data_dir):
        self.data_dir = data_dir
        self.effector2world()
        self.object2cam()
        self.check_and_save()

    def effector2world(self):
        image_file_names = scipy.io.loadmat(os.path.join(self.data_dir, 'imageFileNames'))['imageFileNames'][0] 
        image_order_idx = []
        for f_name in image_file_names:
            image_order_idx.append(int(f_name[0].split('/')[-1].split('.')[0]))

        robot_poses = []
        for img_idx in image_order_idx:
            img_path = os.path.join(self.data_dir, f'{img_idx}_robot_pose.txt')
            with open(img_path, 'r') as f:
                content = f.read().strip()
                digits = content.split(',')
                if len(digits) == 7:
                    robot_poses.append([float(digit) for digit in digits])
                else:
                    print(f"Warning: File {img_path} does not contain exactly 7 comma-delimited values.")
        self.effector2world_transform = np.array(robot_poses)

    def object2cam(self):
        # Pattern pose from camera (Extrinsics)
        # Calibration result loaded from Matlab cameraCalibrator generated script
        calib_result = scipy.io.loadmat(os.path.join(self.data_dir, 'calib_result')) 
        pattern_rotation = calib_result['rotation'].transpose(2,0,1)
        pattern_translation = calib_result['translation']/1000.0
        
        # Convert to quaternions and save to single text file for visp_handeye_cablibrator
        for i in range(len(pattern_rotation)):
            pattern_rotation[i] = pattern_rotation[i].T    
        r = R.from_matrix(pattern_rotation)
        quat = r.as_quat()
        self.object2cam_transform = np.concatenate((pattern_translation.T, quat.T), axis=0).T

    def check_and_save(self):
        if self.effector2world_transform.shape == self.object2cam_transform.shape:
            np.savetxt(os.path.join(self.data_dir, 'effector2world_transform.txt'), self.effector2world_transform, delimiter=', ', fmt='%f')
            np.savetxt(os.path.join(self.data_dir, 'object2cam_transform.txt'), self.object2cam_transform, delimiter=', ', fmt='%f')
            print('Done')
        else:
            print('Number of poses not equal. Not saving poses.')
            print(self.effector2world_transform.shape, self.object2cam_transform.shape)


    @staticmethod
    def transformation_from_rotation_translation(rotation, translation):
        transformations = []
        for r, t in zip(rotation,translation):
            T = np.eye(4)
            T[:3, :3] = r
            T[:3, 3] = t
            transformations.append(T)
        return np.array(transformations)
    
if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Extract data from bag file')
    parser.add_argument('-d', '--data_dir', required=True, type=str, help='path of data directory containing images and robot poses')
    args = parser.parse_args()
    be = PosePairSaver(args.data_dir)

