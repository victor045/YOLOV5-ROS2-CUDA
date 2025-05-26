import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np

from bboxes_ex_msgs.msg import BoundingBoxes, BoundingBox

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        # Load YOLOv5n model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
        if torch.cuda.is_available():
            self.model = self.model.to('cuda')
            self.get_logger().info("Running on CUDA")
        else:
            self.get_logger().info("Running on CPU")

        self.bridge = CvBridge()

        # Subscribe to image topic
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Publish bounding boxes
        self.publisher = self.create_publisher(BoundingBoxes, '/bounding_boxes', 10)

        # Publish annotated image
        self.image_pub = self.create_publisher(Image, '/yolo/image_result', 10)

    def image_callback(self, msg: Image):
        # Convert ROS image to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO detection
        results = self.model(frame)

        # Prepare BoundingBoxes message
        bbox_msg = BoundingBoxes()
        bbox_msg.header = msg.header

        for *xyxy, conf, cls in results.xyxy[0].cpu().numpy():
            bbox = BoundingBox()
            bbox.xmin = int(xyxy[0])
            bbox.ymin = int(xyxy[1])
            bbox.xmax = int(xyxy[2])
            bbox.ymax = int(xyxy[3])
            # bbox.confidence = float(conf)  # Uncomment if added to .msg
            bbox.class_id = str(int(cls))
            bbox_msg.bounding_boxes.append(bbox)

            # Draw the bounding box on the frame
            cv2.rectangle(frame, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (0, 255, 0), 2)
            label = self.model.names[int(cls)]
            cv2.putText(frame, label, (bbox.xmin, bbox.ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
            # After drawing boxes on your OpenCV image (e.g., `frame`)
            cv2.imshow("YOLOv5 Detection", frame)
            cv2.waitKey(1)
            

        # Publish bounding boxes
        self.publisher.publish(bbox_msg)

        # Publish annotated image
        annotated_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        annotated_img_msg.header = msg.header
        self.image_pub.publish(annotated_img_msg)
        
        

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
