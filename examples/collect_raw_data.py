from ur5_controller import URControl as Robot
from laptop_camera import LaptopCamera as Camera
from keyboard_handler import KeyboardHandler
import numpy as np
import click
import os
import cv2
import threading

@click.command()
@click.option('-o', '--output_dir', type=str, required=True)
@click.option('-h', '--hostname', type=str, default='127.0.0.1')
def main(output_dir, hostname):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Directory '{output_dir}' created.")
    else:
        print(f"Directory '{output_dir}' already exists.")

    data_count = 0

    with Camera() as camera, KeyboardHandler() as keyboard_listener:  # Add Robot(hostname) if applicable
        # Start the camera stream in a separate thread
        stream_thread = threading.Thread(target=camera.stream, daemon=True)
        stream_thread.start()
        print("Press 'c' to capture a snapshot or 'q' to quit.")
        while True:
            if keyboard_listener.action == 'capture':
                # Handle capturing a snapshot
                frame = camera.snapshot()
                
                # TODO: Get robot pose: x, y, z, qx, qy, qz
                pose = np.array([0,0,0,0,0,0,0]).reshape(1, -1) # TODO: placeholder

                cv2.imwrite(os.path.join(output_dir, f"{data_count}.jpg"), frame)
                np.savetxt(os.path.join(output_dir, f"{data_count}_robot_pose.txt"), pose, delimiter=",", fmt="%d")
                data_count+=1
                print("Captured a snapshot!")  # Replace with saving/processing logic
                keyboard_listener.action = None  # Reset action
            
            elif keyboard_listener.action == 'freedrive':
                # TODO:
                print(keyboard_listener.action)
                keyboard_listener.action = None  # Reset action
            
            elif keyboard_listener.action == 'stop_freedrive':
                # TODO:
                print(keyboard_listener.action)
                keyboard_listener.action = None  # Reset action

            elif keyboard_listener.action == 'quit':
                print("Exiting...")
                break
            # Other loop operations, e.g., robot control, monitoring, etc.
        stream_thread.join()

if __name__ == "__main__":
    main()