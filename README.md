# Step by Step Calibration Process

1. Collect pairs of images of checkerboard and robot poses. Place them in a data folder, (e.g. ```handeye_calibration/data/sample_data```) 
    - Images should be saved as ```{}.jpg``` where ```{}``` increments by 1. (E.g. ```0.jpg```, ```1.jpg```, ...)
    - Robot poses should be saved as ```{}_robot_pose.txt``` with corresponding index as the iamge. The poses should be comma delimited in the form of: **x,y,z,qx,qy,qz,qw**

2. Open MATLAB and run:
```
cameraCalibrator
```

3. Add all the images in the Camera Calibrator GUI

4. Run ```Calibrate``` once you have checked that all the corners are detected correctly.

5. Visually check the reprojections. 

6. Under ```Export Camera Parameters```, select ```Generate MATLAB script```.

7. Copy the following lines of code to the generated script and run it inside the ```handeye_calibration/data/sample_data``` directory. Before running it, make sure you add the below lines at the end of the script.
```
% ADD THIS TO GENERATED SCRIPT FROM CAMERA CALIBRATOR
save('imagePoints.mat', 'imagePoints')
rotation = cameraParams.RotationMatrices;
translation = cameraParams.TranslationVectors;
save('calib_result.mat', 'rotation', 'translation')
save('imageFileNames.mat', 'imageFileNames')
focal_length = cameraParams.Intrinsics.FocalLength
principal_point = cameraParams.Intrinsics.PrincipalPoint
image_size = cameraParams.Intrinsics.ImageSize
radial_distortion  = cameraParams.Intrinsics.RadialDistortion
tangential_distortion = cameraParams.Intrinsics.TangentialDistortion
skew = cameraParams.Intrinsics.Skew
K = cameraParams.Intrinsics.K
save('camera_params.mat', 'focal_length', "principal_point", ...
    "image_size", "radial_distortion", "tangential_distortion", "skew", "K")
square_size = squareSize
board_dim = detector.BoardSize
save('board_params.mat', 'board_dim', 'square_size')
```

8. Run pose saving script which saves ```effector2world_transform.txt``` and ```object2cam_trasnform.txt``` in the form of: **x,y,z,qx,qy,qz,qw**:
```
python scripts/pose_pair_saver.py -d data/sample_data
```

9. We will now use [VISP](http://wiki.ros.org/visp_hand2eye_calibration) for the extrinsic calibration. For that, we need to use Docker because ROS1 support ended. First, pull the official ROS1 Docker image from the OSRF (Open Source Robotics Foundation) repository:
```
docker pull osrf/ros:noetic-desktop-full
```

10. Start a ROS1 container while mounting the handeye_calibration module. Make sure your terminal is in the handeye_calibration module before runing this command. 
```
docker run -it -v $(pwd):/handeye_calibration osrf/ros:noetic-desktop-full bash
```

11. Once inside the container, update the package lists and install the ros-noetic-vision-visp package:
```
apt-get update
apt-get install -y ros-noetic-vision-visp
source /opt/ros/noetic/setup.bash
```
This will install the ViSP-related packages for ROS Noetic inside your Docker container.
After installation, you can use the ViSP packages in your ROS projects within the container. Remember to source the ROS setup file.

12. Run 2 additiona ROS nodes by opening 2 new terminal windows and connecting to the same container.

Find the container name:
```
docker ps -l
```
Start a new bash session in the same container:
```
docker exec -it <container_name> bash
```
Set up the ROS environment in the new session:
```
source /opt/ros/noetic/setup.bash
```

13. On the first container, run roscore:
```
roscore
```

14. On the second container, run visp hand2eye calibration node:
```
rosrun visp_hand2eye_calibration visp_hand2eye_calibration_calibrator
```

15. From the third container, publish the saved pose pairs:
```
cd handeye_calibration
python3 scripts/transform_publisher.py -d data/sample_data
```

16. Again from the third container, compute handeye transformation matrix by calling service:
```
rosservice call /compute_effector_camera > /handeye_calibration/data/sample_data/hand2eye_transform.yaml
```
This should return you a transformation which is the pose of the camera with respect to the end-effector.
E.g.:
```
effector_camera: 
  translation: 
    x: 0.33626606321608743
    y: -0.46412373894734854
    z: 0.025891346249388294
  rotation: 
    x: -0.5130815365384321
    y: 0.4959403836156112
    z: 0.4977879383736763
    w: 0.4929479091874179
```

17. To check the validity and quality of the calibration, you can reproject detected corners on another image used during the calibration:
```
python scripts/reproject_calibration_results.py -d data/sample_data
```

# Using VISP on Docker

ROS1 support for Unbuntu 22.04 and after. Hence, we will use docker container to use the [visp_hand2eye_calibration](https://wiki.ros.org/visp_hand2eye_calibration) package.

## To run a ROS1 Docker image, follow these steps:

### 1. Pull the ROS1 Docker Image
First, pull the official ROS1 Docker image from the OSRF (Open Source Robotics Foundation) repository:
```
docker pull osrf/ros:noetic-desktop-full
```
This command downloads the full ROS Noetic desktop image

### 2. Run the ROS1 Container
To start a ROS1 container, use the following command:
```
docker run -it osrf/ros:noetic-desktop-full
```
This command starts an interactive session with the running container.

### 3. Set Up the ROS Environment
Once inside the container, set up the ROS environment by sourcing the setup script:
```
source /opt/ros/noetic/setup.bash
```

### 4. Start ROS Core
To start the ROS master, run:
```
roscore
```
This will initiate the ROS core process.

### 5. Running Additional Nodes
To run additional ROS nodes or commands, you'll need to open new terminal windows and connect to the same container. Use the following steps:

Find the container name:
```
docker ps -l
```
Start a new bash session in the same container:
```
docker exec -it <container_name> bash
```
Set up the ROS environment in the new session:
```
source /opt/ros/noetic/setup.bash
```

## The docker image won't have visp installed by default. You can install the ros-noetic-vision-visp package in a Docker container running ROS Noetic. 
Here's how you can do it:

1. Start with a ROS Noetic Docker image:
```
docker pull osrf/ros:noetic-desktop-full
```
2. Run the container in interactive mode:
```
docker run -it osrf/ros:noetic-desktop-full
```
3. Once inside the container, update the package lists:
```
apt-get update
```
4. Install the ros-noetic-vision-visp package:
```
apt-get install -y ros-noetic-vision-visp
```
This will install the ViSP-related packages for ROS Noetic inside your Docker container.
After installation, you can use the ViSP packages in your ROS projects within the container. Remember to source the ROS setup file if you haven't already:
```
source /opt/ros/noetic/setup.bash
```


