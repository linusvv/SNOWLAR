import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
import time
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32

class CompressedImageViewer(Node):
    def __init__(self):
        super().__init__('compressed_image_viewer')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/olive/camera/camera/image/compressed',
            self.image_callback,
            qos_profile_sensor_data)
        self.publisher_ = self.create_publisher(Int32, 'detected_tag_id', 10)
        self.subscription  # prevent unused variable warning
        self.start_time = time.time()
        self.frame_count = 0
        self.fps = 0

        # Set up the window for fullscreen or maximizable
        cv2.namedWindow("Compressed Image Window", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Compressed Image Window", 640, 480)  # Optional: Set an initial window size

        # Load the dictionary that was used to generate the markers.
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()

    def image_callback(self, msg):
        self.frame_count += 1

        if self.frame_count >= 10:
            end_time = time.time()
            elapsed_time = end_time - self.start_time
            self.fps = self.frame_count / elapsed_time
            self.frame_count = 0
            self.start_time = time.time()

        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Convert to OpenCV image

        if cv_image is None:
            self.get_logger().error('Failed to decode image')
            return

        # Display the received image
        cv2.imshow("Compressed Image Window", cv_image)
        cv2.waitKey(1)

        # Convert the image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers in the image
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        # If markers are detected
        if ids is not None:
            self.get_logger().info(f'Detected markers: {ids.flatten()}')
            for i in range(len(ids)):
                # Draw marker borders
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                # Get the ID of the detected marker
                marker_id = ids[i][0]
                # Publish the ID of the detected marker
                msg = Int32()
                msg.data = marker_id
                self.publisher_.publish(msg)
                # Display the ID on the image
                cv2.putText(cv_image, f"ID: {marker_id}", (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        
        # Display FPS on the image
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(cv_image, f"FPS: {self.fps:.2f}", (10, 30), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow("Compressed Image Window", cv_image)

        # Close the window when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()
            exit()

def main(args=None):
    rclpy.init(args=args)
    compressed_image_viewer = CompressedImageViewer()
    rclpy.spin(compressed_image_viewer)

    compressed_image_viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
