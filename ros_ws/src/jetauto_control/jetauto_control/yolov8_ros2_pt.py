from ultralytics import YOLO
import rclpy
import numpy as np
import cv2
import torch
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        self.model = YOLO('yolov8s.pt')
        self.conf = 0.5
        self.min_similarity = 0.7
        self.cx, self.cy = 320, 192
        self.fx = self.fy = 221
        np.random.seed(42)
        self.palette = np.random.randint(0, 255, size=(80, 3), dtype="uint8")

        self.left_subscription = self.create_subscription(
            Image,
            'jetauto/detp_camera/img_raw',
            self.l_camera_callback,
            10)
        self.left_subscription 
        self.right_subscription = self.create_subscription(
            Image,
            'jetauto/detp_camera/img_raw2',
            self.r_camera_callback,
            10)
        self.right_subscription

        self.img_pub = self.create_publisher(Image, "/inference_result", 1)
        self.l_img, self.r_img = None, None

    def l_camera_callback(self, data):
        if self.r_img is None:
            self.l_img = bridge.imgmsg_to_cv2(data, "bgr8")
            
    def r_camera_callback(self, data):
        if self.l_img is not None:
            self.r_img = bridge.imgmsg_to_cv2(data, "bgr8")
            l_boxes, distances, cls_names, colors = self.stereo_matching()
            annotated_image = self.draw_boxes_on_image(self.l_img, l_boxes, distances, cls_names, colors)
            img_msg = bridge.cv2_to_imgmsg(annotated_image) 
            self.img_pub.publish(img_msg)
            self.l_img = None
            self.r_img = None

    def stereo_matching(self):
        l_result = self.model(self.l_img, conf=self.conf)[0]
        r_result = self.model(self.r_img, conf=self.conf)[0]
        l_boxes = l_result.boxes.xyxy.to('cpu').detach().numpy().astype(np.uint16)
        r_boxes = r_result.boxes.xyxy.to('cpu').detach().numpy().astype(np.uint16)

        distances = []
        cls_names = []
        colors = []
        for i, lb in enumerate(l_boxes):
            idx = -1
            max_similarity = 0
            lx1,ly1,lx2,ly2 = lb
            cropped_l_img = self.l_img[ly1:ly2+1, lx1:lx2+1, :]
            cropped_l_img = cv2.resize(cropped_l_img, (50,50))
            for j,rb in enumerate(r_boxes):
                rx1, ry1, rx2, ry2 = rb
                cropped_r_img = self.r_img[ry1:ry2+1, rx1:rx2+1, :]
                cropped_r_img = cv2.resize(cropped_r_img, (50,50))
                similarity = self.cal_similarity(cropped_l_img, cropped_r_img)
                if similarity > self.min_similarity and similarity > max_similarity:
                    max_similarity = similarity
                    sim_rb = rb
                    idx = j

            if idx > -1:
                l_mid_point = (lb[:2] + lb[2:]) / 2
                rb = r_boxes[idx]
                rx1, ry1, rx2, ry2 = sim_rb
                if self.cx > l_mid_point[0]:
                    l_edge_x = lx2
                    r_edge_x = rx2
                else:
                    l_edge_x = lx1
                    r_edge_x = rx1
                if self.cy > l_mid_point[1]:
                    l_edge_y = ly2
                    r_edge_y = ry2
                else:
                    l_edge_y = ly1
                    r_edge_y = ry1
                l_edge_point = (l_edge_x, l_edge_y)
                r_edge_point = (r_edge_x, r_edge_y)
                distance = self.cal_triangulation(l_edge_point, r_edge_point)
            else:
                distance = -1
            distances.append(distance)
            cls_num = int(l_result.boxes.cls[i].cpu())
            cls_names.append(l_result.names[cls_num])
            colors.append(self.palette[cls_num])
        return l_boxes, distances, cls_names, colors

    def cal_similarity(self, img1, img2):
        hist1 = cv2.calcHist([img1], [0], None, [256], [0, 256])
        hist2 = cv2.calcHist([img2], [0], None, [256], [0, 256])
        print(hist1.shape)
        cv2.normalize(hist1, hist1, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
        cv2.normalize(hist2, hist2, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)

        similarity = cv2.compareHist(hist1, hist2, cv2.HISTCMP_CORREL)
        return similarity
    
    def cal_triangulation(self, edge_point1, edge_point2, b=0.2):
        ul, vl = edge_point1
        ur, vr = edge_point2
        x = b*(ul-self.cx) / (ul-ur)
        y = b*self.fx*(vl-self.cy) / (self.fy*(ul-ur))
        z = b*self.fx / (ul-ur)

        real_position = (x,y,z)
        distance = np.linalg.norm(real_position)
        return distance
    
    def draw_boxes_on_image(self, image, boxes, distances,class_names, colors):
        for box, distance, class_name, color in zip(boxes, distances, class_names, colors):
            x_min, y_min, x_max, y_max = box
            color = (int(color[0]),int(color[1]),int(color[2]))
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color, 2)

            if distance < 0:
                label = f'{class_name}: Unknown'
            else:
                label = f'{class_name}: {distance:.2f}'
            cv2.putText(image, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return image
    
# if __name__ == '__main__':
#     rclpy.init(args=None)
#     camera_subscriber = Camera_subscriber()
#     rclpy.spin(camera_subscriber)
#     rclpy.shutdown()

def main(args=None):
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
