from ur5_controller import URControl as Robot
from realsense_camera import RealsenseCamera  as Camera
from keyboard_handler import KeyboardHandler
import numpy as np
import click
import os
import cv2
import threading
from scipy.spatial.transform import Rotation as R

@click.command()
@click.option('-o', '--output_dir', type=str, required=True)
@click.option('-h', '--hostname', type=str, default='192.168.1.10')
@click.option('-r', '--realsense_id', type=str, default='750612070558')
def main(output_dir, hostname, realsense_id):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Directory '{output_dir}' created.")
    else:
        print(f"Directory '{output_dir}' already exists.")

    data_count = 0

    with Camera(realsense_id) as camera, KeyboardHandler() as keyboard_listener, Robot(hostname) as robot:
        # Start the camera stream in a separate thread
        stream_thread = threading.Thread(target=camera.stream, daemon=True)
        stream_thread.start()
        print("Press 'c' to capture a snapshot or 'q' to quit.")
        while True:
            if keyboard_listener.action == 'capture':
                # Handle capturing a snapshot
                frame = camera.snapshot()
                
                # TODO: Get robot pose: x, y, z, qx, qy, qz
                tcp_pose = robot.get_tcp_pose()
                translation = tcp_pose[:3]
                rot_vec = tcp_pose[3:]
                rot = R.from_rotvec(rot_vec)
                quaternion = rot.as_quat()
                pose = np.concatenate([translation, quaternion]).reshape(1, -1)
                cv2.imwrite(os.path.join(output_dir, f"{data_count}.jpg"), frame)
                np.savetxt(os.path.join(output_dir, f"{data_count}_robot_pose.txt"), pose, delimiter=",", fmt="%.8f")
                data_count+=1
                print("Captured a snapshot!")  # Replace with saving/processing logic
                keyboard_listener.action = None  # Reset action
            
            elif keyboard_listener.action == 'freedrive':
                # TODO:
                robot.activate_teach_mode()
                print(keyboard_listener.action)
                keyboard_listener.action = None  # Reset action
            
            elif keyboard_listener.action == 'stop_freedrive':
                # TODO:
                robot.deactivate_teach_mode()
                print(keyboard_listener.action)
                keyboard_listener.action = None  # Reset action

            elif keyboard_listener.action == 'quit':
                print("Exiting...")
                break
            # Other loop operations, e.g., robot control, monitoring, etc.
    stream_thread.join()

if __name__ == "__main__":
    main()