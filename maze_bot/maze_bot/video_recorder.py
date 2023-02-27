import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os


class GetVideo(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.subscriber = self.create_subscription(Image, '/top_camera/image_raw', self.process_data, 10) 
        video_path = os.path.join(os.getcwd(), 'output.avi')  # The path to where to save the video.
        # 'M', 'J', 'P', 'G' is the FourCC code for the video codec, 30 fps, 1280X720 frame size in pixels.
        self.out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30, (1280,720))
        self.bridge = CvBridge()  # Convert the ros images to openCV data.

    
    def process_data(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.out.write(frame)         # Write the frame to the video file.
        cv2.imshow('output', frame)   # Display the frame.
        cv2.waitKey(1)


def main():
    rclpy.init()
    img_sub = GetVideo()
    rclpy.spin(img_sub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
