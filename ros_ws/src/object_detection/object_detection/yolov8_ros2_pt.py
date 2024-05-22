import rclpy.logging
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path
from yolov8_msgs.msg import InferenceResult, Yolov8Inference
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np

bridge = CvBridge()


class Camera_subscriber(Node):

    def __init__(self):
        super().__init__("object_detection_node")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.UNSET)

        self.model = YOLO(
            Path(
                get_package_share_directory("object_detection"),
                "resource/model/yolov8n.pt",
            ),
            verbose=False
        )

        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image, "jetauto/detp_camera/img_raw", self.camera_callback, 10
        )

        # self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

        self.pallete = np.random.randint(0, 255, size=(80, 3), dtype="uint8")

    def camera_callback(self, data):

        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(img)

        self.yolov8_inference.header.frame_id = "inference"
        # self.yolov8_inference.header.stamp = camera_subscriber.get_clock().now().to_msg()
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = (
                    box.xyxy[0].to("cpu").detach().numpy().copy()
                )  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                self.yolov8_inference.yolov8_inference.append(self.inference_result)

        # annotated_frame = results[0].plot()
        annotated_frame = self.draw_cls(img, results[0])
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)

        self.img_pub.publish(img_msg)
        # self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

    # 이미지에 찾은 객체에 네모 사각형과 텍스트 출력
    def draw_cls(self, img: object, result: object):

        boxes = result.boxes
        cls_names = result.names
        for cord, cls, conf in zip(
            boxes.xyxy.tolist(),
            boxes.cls.tolist(),
            boxes.conf.tolist(),
        ):
            color = tuple(map(lambda x: int(x), self.pallete[int(cls)]))
            cord = tuple(map(lambda x: int(x), cord))
            cv2.rectangle(
                img,
                cord[0:2],
                cord[2:4],
                color,
                2,
            )
            cv2.putText(
                img,
                f"{cls_names[int(cls)]}:{conf:.3f}",
                (cord[0], cord[1] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
            )
        return img


def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
