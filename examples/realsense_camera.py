import pyrealsense2 as rs
import numpy as np
import cv2
import imageio

class RealsenseCamera:
    def __init__(self, id):
        self.id = id

        self.fps = 30
        self.width = 640
        self.height = 480
        
        # Configure depth and color streams...
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(self.id)
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)

        # Start streaming 
        self.camera = self.pipeline.start(self.config)
        self.intrinsic_parameters = self.get_rgb_intrinsics()

    def get_rgb_intrinsics(self):
        profile = self.camera.get_stream(rs.stream.color) # Fetch stream profile for depth stream
        intr = profile.as_video_stream_profile().get_intrinsics()    
        return {'fx': intr.fx, 'fy': intr.fy, 'cx': intr.ppx, 'cy': intr.ppy}


    def stream(self):
        try:
            while True:
                frame = self.pipeline.wait_for_frames()
                color_frame = frame.get_color_frame()
                if  not color_frame:
                    continue

                # Convert images to numpy arrays
                rgb_arr = np.asanyarray(color_frame.get_data())

                # Show images
                cv2.imshow('RealSense', rgb_arr)
                # Break the loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            # Stop streaming
            cv2.destroyAllWindows()

    def snapshot(self):
        frame = self.pipeline.wait_for_frames()
        color_frame = frame.get_color_frame()
        if not color_frame:
            print('Error: Could not read frame.')
            return None
        else:
            # Convert images to numpy arrays
            return np.asanyarray(color_frame.get_data())
            
    def record(self, n_frames):
        frame_count = 0
        writer = imageio.get_writer('cam.mp4', fps=self.fps)
        try:
            while frame_count < n_frames:
                frame = self.pipeline.wait_for_frames()
                color_frame = frame.get_color_frame()
                if  not color_frame:
                    continue

                # Convert images to numpy arrays
                rgb_arr = np.asanyarray(color_frame.get_data())

                # Show images
                cv2.imshow('RealSense', rgb_arr)
                cv2.waitKey(1)

                writer.append_data(cv2.cvtColor(np.copy(rgb_arr), cv2.COLOR_BGR2RGB))
                frame_count+=1
        finally:
            # Stop streaming
            self.close()
            writer.close()

    def close(self):
        self.pipeline.stop()

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        if exc_type:
            print(f"An exception occurred: {exc_value}")

if __name__=='__main__':
    cam = RealsenseCamera('750612070558')
    print(cam.intrinsic_parameters)
    #cam.record(500)