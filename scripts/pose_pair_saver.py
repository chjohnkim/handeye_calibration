##!/usr/bin/env python
import os
import yaml
import scipy.io
import numpy as np
from scipy.spatial.transform import Rotation as R

class PosePairSaver():
    def __init__(self, data_root_dir, bag_dir):
        self.data_root_dir = data_root_dir
        self.bag_dir = bag_dir
        self.effector2world()
        self.object2cam()
        print('Done')

    def effector2world(self):
        # End-effector pose from robot base/world frame
        filename = os.path.join(self.data_root_dir, self.bag_dir, 'pose_data.yaml')
        yaml_dict = yaml.load(open(filename), Loader=yaml.CLoader)
        robot_pose = []
        for key in sorted(yaml_dict.keys()):
            q = yaml_dict[key]['quaternion']
            t = yaml_dict[key]['translation']
            robot_pose.append([t['x'], t['y'], t['z'], q['x'], q['y'], q['z'], q['w']])
            
        robot_pose = np.array(robot_pose)

        # Save single text file with all relevant robot poses for visp_handeye_calibrator 
        np.savetxt(os.path.join(self.data_root_dir, self.bag_dir, 'effector2world_transform.txt'), robot_pose, delimiter=', ', fmt='%f')

        # Construct Nx4x4 matrix containing homogenous transformation matrices for reprojection purposes
        robot_translation = []
        robot_rotation = []
        for pose in robot_pose:
            robot_translation.append(pose[:3])
            r = R.from_quat(pose[3:])
            robot_rotation.append(r.as_matrix())
        robot_translation = np.array(robot_translation)
        robot_rotation = np.array(robot_rotation)
        T_effector_world = self.transformation_from_rotation_translation(robot_rotation, robot_translation)

    def object2cam(self):
        # Pattern pose from camera (Extrinsics)
        # Calibration result loaded from Matlab cameraCalibrator generated script
        calib_result = scipy.io.loadmat(os.path.join(self.data_root_dir, self.bag_dir, 'calib_result')) 
        pattern_rotation = calib_result['rotation'].transpose(2,0,1)
        pattern_translation = calib_result['translation']/1000.0

        # Construct Nx4x4 homogenious transformation matrices for reprojetion
        T_pattern_camera = self.transformation_from_rotation_translation(pattern_rotation, pattern_translation)
        
        # Convert to quaternions and save to single text file for visp_handeye_cablibrator
        for i in range(len(pattern_rotation)):
            pattern_rotation[i] = pattern_rotation[i].T    
        r = R.from_matrix(pattern_rotation)
        quat = r.as_quat()
        object2cam_transform = np.concatenate((pattern_translation.T, quat.T), axis=0)
        np.savetxt(os.path.join(self.data_root_dir, self.bag_dir, 'object2cam_transform.txt'), object2cam_transform.T, delimiter=', ', fmt='%f')

    @staticmethod
    def transformation_from_rotation_translation(rotation, translation):
        transformations = []
        for r, t in zip(rotation,translation):
            T = np.eye(4)
            T[:3, :3] = r
            T[:3, 3] = t
            transformations.append(T)
        return np.array(transformations)