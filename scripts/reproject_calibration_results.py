import scipy.io
import numpy as np
import os
from scipy.spatial.transform import Rotation as R
import cv2
import matplotlib.pyplot as plt
import argparse
import yaml

class ReprojectHand2Eye:
    def __init__(self, data_dir):
        self.data_dir = data_dir
        self.load_camera_params()
        self.load_calibration_board_params()
        self.load_hand2eye_transformation()
        self.T_effector_world = self.load_transformation_pairs('effector2world_transform.txt')
        self.T_pattern_camera = self.load_transformation_pairs('object2cam_transform.txt')

    def load_camera_params(self):
        camera_params = scipy.io.loadmat(os.path.join(self.data_dir, 'camera_params'))
        radial_distortion = camera_params['radial_distortion'][0]
        tangential_distortion = camera_params['tangential_distortion'][0]
        self.K = camera_params['K']
        self.distortion = np.array([radial_distortion[0], radial_distortion[1], tangential_distortion[0], tangential_distortion[1], 0])

    def load_calibration_board_params(self):
        board_params = scipy.io.loadmat(os.path.join(self.data_dir, 'board_params'))
        self.board_size = board_params['board_dim'][0][::-1]-1
        self.square_size = board_params['square_size'].item()/1000.0

    def load_hand2eye_transformation(self):
        # Camera pose from end-effector frame
        yaml_path = os.path.join(self.data_dir, 'hand2eye_transform.yaml')
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)
        x = data['effector_camera']['translation']['x']
        y = data['effector_camera']['translation']['y']
        z = data['effector_camera']['translation']['z']
        qx = data['effector_camera']['rotation']['x']
        qy = data['effector_camera']['rotation']['y']
        qz = data['effector_camera']['rotation']['z']
        qw = data['effector_camera']['rotation']['w']
        camera_translation = np.array([[x, y, z]])
        camera_rotation = R.from_quat([qx, qy, qz, qw])
        T_camera_effector = np.eye(4)
        T_camera_effector[:3,3] = camera_translation
        T_camera_effector[:3,:3] = camera_rotation.as_matrix()
        self.T_camera_effector = T_camera_effector 

    def load_transformation_pairs(self, txt_file):
        # Read the CSV file into a NumPy array
        poses = np.loadtxt(os.path.join(self.data_dir, txt_file), delimiter=',')
        # Construct Nx4x4 matrix containing homogenous transformation matrices for reprojection purposes
        translations = []
        rotations = []
        for pose in poses:
            translations.append(pose[:3])
            r = R.from_quat(pose[3:])
            rotations.append(r.as_matrix())
        translations = np.array(translations)
        rotations = np.array(rotations)
        T = self.transformation_from_rotation_translation(rotations, translations)
        return T

    @staticmethod
    def transformation_from_rotation_translation(rotation, translation):
        transformations = []
        for r, t in zip(rotation,translation):
            T = np.eye(4)
            T[:3, :3] = r
            T[:3, 3] = t
            transformations.append(T)
        return np.array(transformations)

    @staticmethod
    def inv(T):
        T_inv = np.eye(4)
        T_inv[:3,:3] = np.linalg.inv(T[:3,:3])
        T_inv[:3,3] = -np.matmul(np.linalg.inv(T[:3,:3]), T[:3,3]) 
        return T_inv    
    
    @staticmethod
    def calculate_rmse(predicted, actual):
        return np.sqrt(np.mean((predicted - actual)**2))

    def reproject(self, save_figures):
        imagePoints = scipy.io.loadmat(os.path.join(self.data_dir, 'imagePoints'))['imagePoints']

        # Chess coordinates in chess frame
        point_coordinates = []
        for x in range(self.board_size[0]):
            for y in range(self.board_size[1]):
                point_coordinates.append([x*self.square_size, y*self.square_size, 0, 1])
        points = np.array(point_coordinates).T

        image_file_names = scipy.io.loadmat(os.path.join(self.data_dir, 'imageFileNames'))['imageFileNames'][0] 
        image_id = []
        for f_name in image_file_names:
            image_id.append(int(f_name[0].split('/')[-1].split('.')[0]))
        imgs = []
        for i in image_id:
            imgs.append(cv2.imread(os.path.join(self.data_dir, f'{i}.jpg')))

        # Homogenious transformation matrices
        T_cb = np.array(self.T_pattern_camera) # T_cb: board in camera frame (Nx4x4)
        T_ec = np.array(self.T_camera_effector) # T_ec: camera in end-effector frame (4x4)
        T_we = np.array(self.T_effector_world) # T_we: end-effector in world frame (Nx4x4)

        # Camera projection matrix: P=K[R|t]
        Rt = np.array([[1,0,0,0],
                    [0,1,0,0],
                    [0,0,1,0]])
        P = np.matmul(self.K,Rt)

        reprojected_points = np.empty([2, 0])
        original_points = np.empty([2, 0])
        for i in range(0,len(image_id)):
            # undistort image
            undistorted_img = cv2.undistort(imgs[i], self.K, self.distortion, None, self.K)
            undistorted_img = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2RGB)
            # undistort points detected in chessboard
            image_points = cv2.undistortPoints(imagePoints[:,:,i], self.K, self.distortion)
            image_points = image_points.transpose(2,1,0).reshape(2,-1) 
            # applying necessary fix to account for undistortion
            image_points[0] = image_points[0]*self.K[0,0]+self.K[0,2]
            image_points[1] = image_points[1]*self.K[1,1]+self.K[1,2] 
            for j in range(0,len(image_id)):
                if i==j:
                    continue
                # Points in j'th board frame transformed and reprojected into i'th image 
                points_j_in_camera_j = np.matmul(T_cb[j], points)
                points_j_in_ee_j = np.matmul(T_ec, points_j_in_camera_j)
                points_j_in_world = np.matmul(T_we[j], points_j_in_ee_j)
                points_j_in_ee_i = np.matmul(self.inv(T_we[i]), points_j_in_world)
                points_j_in_camera_i = np.matmul(self.inv(T_ec), points_j_in_ee_i)
                
                reprojection = np.matmul(P, points_j_in_camera_i)
                reprojection[0]/=reprojection[2] 
                reprojection[1]/=reprojection[2] 
                reprojection[2]/=reprojection[2] 

                reprojected_points = np.column_stack([reprojected_points, reprojection[:2]])
                original_points = np.column_stack([original_points, image_points])

                if save_figures:                   
                    plt.figure(figsize=(15,15))
                    plt.scatter(image_points[0], image_points[1], marker='o', s=80, linewidths=1, facecolors='none', edgecolors='g')
                    plt.plot(reprojection[0],reprojection[1], 'rx', markersize=8)
                    plt.imshow(undistorted_img)
                    output_dir = os.path.join(self.data_dir, 'out_reprojection') 
                    if not os.path.exists(output_dir):
                        os.makedirs(output_dir)
                        print(f"Directory '{output_dir}' created successfully.")
                    plt.savefig(os.path.join(output_dir, '{}_{}.png'.format(i,j)))
                    plt.close()
        original_nan_indices = np.where(np.isnan(original_points) | np.isinf(original_points))[1]
        original_nan_indices = np.array(np.sort(list(set(original_nan_indices))))
        rmse = self.calculate_rmse(reprojected_points.T[~original_nan_indices].T, original_points.T[~original_nan_indices].T)
        print('Pixel Reprojection Error:', rmse)

if __name__=='__main__': 
    
    parser = argparse.ArgumentParser(description='Reproject hand2eye calibration result')
    parser.add_argument('-d', '--data_dir', required=True, type=str, help='path of data directory containing images and robot poses')
    parser.add_argument('-f', '--save_figures', type=bool, default=False, help='Whether to save reprojection figures.')
    args = parser.parse_args()
    be = ReprojectHand2Eye(args.data_dir)
    be.reproject(save_figures=args.save_figures)
