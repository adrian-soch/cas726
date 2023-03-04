import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cas726_interfaces.srv import DetectObjects
from cas726_interfaces.msg import BoundingBox

from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

import torch
import numpy

from torchvision.io.image import read_image
from torchvision.models.detection import fasterrcnn_resnet50_fpn_v2, FasterRCNN_ResNet50_FPN_V2_Weights
from torchvision.utils import draw_bounding_boxes
from torchvision.transforms.functional import to_pil_image
import torchvision.transforms as transforms

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('object_detector')
        print("Creating service")
        self.detect_obj_srv = self.create_service(
            DetectObjects,
            'object_detector/detect',
            self.detect_callback)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Step 1: Initialize model with the best available weights
        self.weights = FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT
        self.model = fasterrcnn_resnet50_fpn_v2(weights=self.weights, box_score_thresh=0.9)
        self.model.eval()
        # Step 2: Initialize the inference transforms
        self.preprocess = self.weights.transforms()
        print("Creating service --> done")

    def detect_callback(self, request, response):
        print("Processing image")
        self.get_logger().info('Got image in frame: "%s"' % request.color.header.frame_id)

        #TODO: implement this function.
        # 1. convert request image to a tensor. Go via the opencv bridge and then convert to tensor
        # 2. create a minibatch with just one image and call self.model to do inference
        # 3. iterate through predictions and copy data into the response message

        print("Done")
        return response


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
