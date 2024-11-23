import cv2

class LaptopCamera:
    def __init__(self):
        # Start streaming 
        self.camera = cv2.VideoCapture(0)  # Use 0 for the default camera. For external cameras, use 1 or the appropriate index.
        # Check if the camera opened successfully
        if not self.camera.isOpened():
            print("Error: Could not open camera.")
            exit()

    def stream(self):
        # Loop to continuously capture frames
        while True:
            ret, frame = self.camera.read()  # Capture a frame

            if not ret:
                print("Error: Could not read frame.")
                break

            # Display the frame
            cv2.imshow('Camera Feed', frame)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Release the camera and close windows
        cv2.destroyAllWindows()

    def snapshot(self):
        ret, frame = self.camera.read()  # Capture a frame
        if not ret:
            print("Error: Could not read frame.")
            return None
        else:
            return frame
    
    def close(self):
        self.camera.release()

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        if exc_type:
            print(f"An exception occurred: {exc_value}")

if __name__=='__main__':
    with LaptopCamera() as cam:
        cam.snapshot()
    #print(cam.intrinsic_parameters)
    #cam.record(500)