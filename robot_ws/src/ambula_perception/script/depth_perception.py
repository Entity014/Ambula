#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
from message_filters import ApproximateTimeSynchronizer, Subscriber

class AmbulaDataCollector(Node):
    def __init__(self):
        super().__init__('ambula_data_collector')
        self.bridge = CvBridge()
        
        session_id = time.strftime("%Y%m%d_%H%M%S")
        self.save_path = f'dataset_ambula_{session_id}'
        
        os.makedirs(f'{self.save_path}/rgb', exist_ok=True)
        os.makedirs(f'{self.save_path}/depth_color', exist_ok=True)

        self.rgb_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')

        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

        self.get_logger().info(f'--- Data Collector Started ---')
        self.get_logger().info(f'Saving to: {self.save_path}')

    def callback(self, rgb_msg, depth_msg):
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            cv_depth_16 = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

            # 1. ทำ Colorized Depth (0-4m)
            depth_norm = cv2.convertScaleAbs(cv_depth_16, alpha=255.0/4000.0) 
            cv_depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

            # 2. บันทึกภาพต้นฉบับ (บันทึกขนาดเต็ม 640x480 เพื่อให้ Model เทรนได้ดี)
            key = cv2.waitKey(1)
            if key == ord('s'):
                ts = int(time.time() * 1000)
                cv2.imwrite(f'{self.save_path}/rgb/img_{ts}.jpg', cv_rgb)
                cv2.imwrite(f'{self.save_path}/depth_color/img_{ts}.jpg', cv_depth_color)
                self.get_logger().info(f'Captured! Frame ID: {ts}')

            # 3. ย่อจอสำหรับ Preview (ย่อเหลือ 50% ของขนาดเดิม)
            # จาก 1280x480 จะเหลือ 640x240 ดูสบายตาขึ้น
            preview_combined = cv2.hconcat([cv_rgb, cv_depth_color])
            
            # คำนวณขนาดใหม่
            width = int(preview_combined.shape[1] * 0.5)
            height = int(preview_combined.shape[0] * 0.5)
            preview_small = cv2.resize(preview_combined, (width, height), interpolation=cv2.INTER_AREA)

            # ใส่ตัวหนังสือบนจอเล็ก
            cv2.putText(preview_small, "[S] Save  [Q] Quit", (10, 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow("Ambula Monitor (Small)", preview_small)

            if key == ord('q'):
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AmbulaDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()