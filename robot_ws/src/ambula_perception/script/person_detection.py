#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class PersonDetectionNode(Node):
    def __init__(self):
        super().__init__('person_detection_node')
        
        # 1. โหลดโมเดล YOLO (แนะนำ yolov8n.pt หรือตัวที่คุณเทรนเอง)
        # ถ้าใช้บน Pi 5 แนะนำให้ทำ Export เป็น OpenVINO จะเร็วขึ้นมาก
        self.model = YOLO('/home/robot_ws/best.pt') 
        
        self.bridge = CvBridge()
        
        # 2. Subscribe ไปที่ Aligned Depth
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.detection_callback,
            10)
            
        self.get_logger().info('YOLO Depth Detection Node has started!')

    def detection_callback(self, msg):
        try:
            # 3. แปลงจาก ROS Image เป็น OpenCV (16-bit)
            cv_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")

            # 4. Pre-process ภาพ Depth ให้เป็น 8-bit เหมือนตอนที่เราเก็บ Dataset
            # ตัดระยะที่ 5 เมตร (ปรับตามความเหมาะสมของ Ambula)
            max_dist = 5000.0 
            depth_8bit = cv2.convertScaleAbs(cv_depth, alpha=(255.0/max_dist))
            
            # แปลงเป็น 3 Channels (RGB) เพื่อให้เข้าโครงสร้างโมเดล YOLO ปกติได้
            input_img = cv2.cvtColor(depth_8bit, cv2.COLOR_GRAY2BGR)

            # 5. รัน Inference (เน้นเฉพาะคลาส 'person' คือ id 0)
            results = self.model.predict(input_img, classes=[0], conf=0.5, verbose=False)

            # 6. วาดผลลัพธ์ลงบนภาพ (Visualization)
            annotated_frame = results[0].plot()

            # 7. ดึงพิกัดเพื่อไปคำนวณระยะทางจริง (ตัวอย่าง)
            for box in results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                center_x, center_y = int((x1+x2)/2), int((y1+y2)/2)
                
                # ดึงระยะจริงจากภาพ 16-bit ณ จุดกึ่งกลาง
                real_dist = cv_depth[center_y, center_x] / 1000.0 # หน่วยเป็นเมตร
                self.get_logger().info(f'Detected Person at: {real_dist:.2f}m')

            # แสดงผลหน้าจอ
            cv2.imshow("YOLO Depth Detection", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Detection Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()