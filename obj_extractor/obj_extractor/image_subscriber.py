import cv2

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from cas726_interfaces.msg import BoundingBox
from cas726_interfaces.srv import DetectObjects

import torchvision.transforms as transforms
from torchvision.models.detection import fasterrcnn_resnet50_fpn_v2, FasterRCNN_ResNet50_FPN_V2_Weights

DEBUG = False # Set to True to visualize detections

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('object_detector')
        print("Creating service")
        self.detect_obj_srv = self.create_service(
            DetectObjects,
            'object_detector/detect',
            self.detect_callback)

        self.br = CvBridge()

        # Initialize model with the best available weights
        self.weights = FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT
        self.model = fasterrcnn_resnet50_fpn_v2(weights=self.weights, box_score_thresh=0.9)
        self.model.eval()

        # Initialize the inference transforms
        self.preprocess = self.weights.transforms()

        # Initialize tranform to tensor
        self.toTensor = transforms.ToTensor() 

        self.get_logger().info('Service created.')

    def detect_callback(self, request, response):
        self.get_logger().info('Got image in frame: "%s"' % request.color.header.frame_id)

        # Convert ROS Image message to OpenCV image
        img = self.br.imgmsg_to_cv2(request.color)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Convert to tensor
        tensor = self.toTensor(img)

        # Apply inference preprocessing transforms
        batch = [self.preprocess(tensor)]

        # Use the model and visualize the detections
        detections = self.model(batch)[0]

        # Get text labels for each detection in image
        labels = [self.weights.meta["categories"][i] for i in detections["labels"]]
        
        response.detections = []
        # loop over the detections
        for i in range(0, len(detections["boxes"])):

            confidence = detections["scores"][i]
            CONF_THRESH = 0.7
            if confidence > CONF_THRESH:

                box = detections["boxes"][i].detach().cpu().numpy()
                (startX, startY, endX, endY) = box.astype("int")

                # fill bbox values
                bbox = BoundingBox()
                bbox.label = labels[i]
                bbox.x_min = int(startX)
                bbox.y_min = int(startY)
                bbox.x_max = int(endX)
                bbox.y_max = int(endY)
                response.detections.append(bbox)
                
                if DEBUG:
                    # draw the bounding box and label on the image
                    cv2.rectangle(img, (bbox.x_min, bbox.y_min), (bbox.x_max, bbox.y_max), (50,220,50), 2)
        if DEBUG:
            cv2.imshow("Output", img)
            cv2.waitKey(5)

        return response


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
