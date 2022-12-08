# Calibration process

1. If bag file is named **hello_world.bag**, place bag file in **handeye_calibration/data/hello_world/hello_world.bag**
2. Run bag extraction script:

```
python extract_bag.py -d data -b hello_world -s 2 -r 20
```

3. Run cameraCalibrator on Matlab with extracted images and generate calibration script.

4. Copy the following lines of code to the generated script and run it inside the **handeye_calibration/data/hello_world** directory. Before running it, make sure you add the below lines at the end of the script.
```
% ADD THIS TO GENERATED SCRIPT FROM CAMERA CALIBRATOR
save('imagePoints.mat', 'imagePoints')
rotation = cameraParams.RotationMatrices;
translation = cameraParams.TranslationVectors;
save('calib_result.mat', 'rotation', 'translation')
```

5. Run pose saving script which saves end-effector2world and object2camera poses to text file in the form of: **x,y,z,qx,qy,qz,qw**
```
python save_poses.py -d data -b hello_world
```

5. Run handeye_calibration ROS package (http://wiki.ros.org/visp_hand2eye_calibration)
```
rosrun visp_hand2eye_calibration visp_hand2eye_calibration_calibrator
```

6. Publish saved poses:
```
python publish_poses.py -d data -b hello_world
```

7. Compute handeye transformation matrix by calling service:
```
rosservice call /compute_effector_camera
```
