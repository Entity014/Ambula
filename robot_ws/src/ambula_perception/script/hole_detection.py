#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from message_filters import ApproximateTimeSynchronizer, Subscriber

class AmbulaRGBSegNode(Node):
    def __init__(self):
        super().__init__('ambula_rgb_seg_node')
        
        # 1. โหลดโมเดล (ต้องมั่นใจว่าตอนเทรนใน Roboflow ใช้ภาพ RGB เทรนด้วย)
        self.model = YOLO('/home/robot_ws/best.pt') 
        self.bridge = CvBridge()
        
        # 2. ตั้งค่า Subscriber สำหรับทั้ง RGB และ Depth
        self.rgb_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')

        # Synchronize ภาพให้ตรงกัน
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.detection_callback)
            
        self.get_logger().info('Ambula RGB Segmentation Node Started!')

    def detection_callback(self, rgb_msg, depth_msg):
        try:
            # 3. แปลงภาพทั้งสอง
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
            height, width = cv_rgb.shape[:2]

            # 4. รัน YOLO บนภาพ RGB ปกติ
            results = self.model.predict(cv_rgb, conf=0.5, verbose=False)
            
            # ใช้ภาพ RGB เป็นภาพพื้นหลังสำหรับวาดผลลัพธ์
            annotated_frame = cv_rgb.copy()

            if results[0].masks is not None:
                # วาด Mask ลงบนภาพ RGB
                annotated_frame = results[0].plot()

                for mask_data in results[0].masks.xy:
                    # หาจุดต่ำสุดของคาน (Lintel)
                    bottom_point = mask_data[mask_data[:, 1].argmax()]
                    
                    px = int(np.clip(bottom_point[0], 0, width - 1))
                    py = int(np.clip(bottom_point[1], 0, height - 1))

                    # 5. ดึงระยะจากภาพ Depth มาคำนวณความสูง
                    roi = cv_depth[max(0, py-2):min(height, py+2), 
                                   max(0, px-2):min(width, px+2)]
                    
                    if roi.size > 0 and np.any(roi > 0):
                        raw_dist = np.mean(roi[roi > 0])
                        dist_to_lintel = raw_dist / 1000.0 # เมตร
                    else:
                        dist_to_lintel = 0.0

                    # วาดจุดมาร์ค
                    cv2.circle(annotated_frame, (px, py), 7, (0, 255, 0), -1) # สีเขียวบนภาพสี
                    cv2.putText(annotated_frame, f"H: {dist_to_lintel:.2f}m", (px, py - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # ย่อจอ Preview 50%
            preview_small = cv2.resize(annotated_frame, (int(width*0.5), int(height*0.5)))
            cv2.imshow("Ambula RGB Vision", preview_small)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Detection Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AmbulaRGBSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()