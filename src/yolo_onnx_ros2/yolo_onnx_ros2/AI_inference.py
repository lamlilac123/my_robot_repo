'''
giới thiệu: đoạn code này chính là sự gộp lại giữa node đọc cam (c++) và node infer AI (python) 

mục đích: 
- hạn chế được những lỗi ở c++ (ngôn ngữ mà tác giả chỉ hiểu đến mức ứng dụng trung bình)
- tránh được độ trễ khi publish cả tấm ảnh từ node đọc cam -> node infer
- ôn lại về ros2 một tí vì lâu chưa code node lại

hỗ trợ: 
- Claude sẽ được dùng để hỏi về cú pháp (API xử lý) phức tạp và nhiều tham số, kèm theo ý tưởng đa luồng 2 thread
- Bên cạnh đó, tác giả cũng sẽ dùng lại các khung sườn và ý tưởng từ node cũ cho tiết kiệm tgian và ôn tập hiệu quả

cải tiến: tối ưu lại code cho đặc thù bài toán nhận diện 1 class duy nhất
''' 

import rclpy
import cv2
import threading
import time

import onnxruntime as ort
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from threading import Thread

import os
from ament_index_python.packages import get_package_share_directory

# Lấy đường dẫn tự động tới file pou.onnx
model_normal_name = "yellow_wheel.onnx"
pkg_share = get_package_share_directory('yolo_onnx_ros2')

MODEL_ONNX_PATH = os.path.join(pkg_share, 'models', model_normal_name)
IP_CAM_URL = "http://192.168.0.175:8081/video" 


class inference_node(Node):
    def __init__(self):
        super().__init__("Inference_Node") 
        self.cap = cv2.VideoCapture(IP_CAM_URL)
        if not self.cap.isOpened():
            self.get_logger().info("\n\n\n\n KHÔNG MỞ ĐƯỢC CAMERA! \n\n\n\n")
            exit()
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.get_logger().info(f"fps thực: {self.cap.get(cv2.CAP_PROP_FPS)}")


        self.model = ort.InferenceSession(MODEL_ONNX_PATH)
        self.input_name = self.model.get_inputs()[0].name

        self.in_w = 320
        self.in_h = 192
        self.max_angle_one_side = 55
        self.base_confidence = 0.25


        self.last_pan_angle = 90
        self.last_tilt_angle = 90

        self.servo_pub_ = self.create_publisher(Int32MultiArray, "servo_angles", 1)
 
        self.latest_frame = None
        self.new_frame = threading.Event()   # dùng bật cờ cho infer

        self.camera_thread = threading.Thread(target=self.read_camera, daemon=True)
        self.camera_thread.start()

        self.inference_thread = threading.Thread(target=self.run_inference, daemon=True)
        self.inference_thread.start()

    def notify_servo_angle(self, xcent, ycent, width, height):
        half_width = width // 2
        half_height = height // 2
        
        #tính erorr: 0 độ nằm gần gốc -> âm thì là giảm góc -> dương là tăng góc
        x_error = xcent - half_width
        y_error = ycent - half_height

        # self.max_angle_one_side là lệch tối đa về 1 phía mà cam có thể thấy dc mà ko cần xoay 
        # dữ liệu đo ước lương và đo đạc từ webcam laptop
        x_angle_error = -1*int((x_error / half_width)*self.max_angle_one_side)
        y_angle_error = -1*int((y_error / half_height)*self.max_angle_one_side)

        x_aim_angle = max(0, min(180, self.last_pan_angle + x_angle_error))
        y_aim_angle = max(0, min(180, self.last_tilt_angle + y_angle_error))

        #self.last_pan_angle = x_aim_angle
        #self.last_pan_angle = y_aim_angle

        #publish lên cho writer
        msg = Int32MultiArray()
        msg.data = [x_aim_angle, y_aim_angle]
        self.servo_pub_.publish(msg)

    def read_camera(self):
        while rclpy.ok():
            ok, frame = self.cap.read()
            if not ok:
                continue
            self.latest_frame = frame   
            self.new_frame.set()

    def run_inference(self):
        while rclpy.ok():
            if not self.new_frame.wait(timeout=1.0):
                continue

            self.new_frame.clear()
            cv_img = self.latest_frame        
            H, W, _ = cv_img.shape

            # infer bằng ort
            t0 = time.perf_counter()
            input_tensor = cv2.dnn.blobFromImage(cv_img, 1/255, (self.in_w, self.in_h), swapRB=True)
            t1 = time.perf_counter()
            outputs = self.model.run(None, {self.input_name:input_tensor})
            outputs = np.squeeze(outputs).T         
            '''
            ouputs:
            - shape = (số_vật_thể, 4(tọa_độ)+số_class) -> sau 4 tọa độ là confidence cho từng class (tổng khác 1)
            - tọa độ: tâm x, tâm y, chiều rộng, chiều cao
            '''
            t2 = time.perf_counter()
            # lấy ra row có confidence cao nhất (vì đặc thù chỉ có 1 object và 1 class cần tìm)
            highest_confidence_row_index = np.argmax(outputs[:, 4])      # chỉ số hàng có confidence cao nhất
            row = outputs[highest_confidence_row_index]                      # [cx, cy, w, h, conf]

            # đổi tọa độ thành xy max-min và xử lý
            if row[4] > self.base_confidence:
                cx, cy, w, h = row[:4]

                # đổi tọa độ từ hệ ảnh input của model sang ảnh gốc
                # in_w, in_h là kích thước input của model
                scale_x = W / self.in_w
                scale_y = H / self.in_h
                cx *= scale_x
                w  *= scale_x
                cy *= scale_y
                h  *= scale_y

                # đổi từ (tâm, rộng, cao) sang (min, max)
                xmin = int(cx - w / 2)
                ymin = int(cy - h / 2)
                xmax = int(cx + w / 2)
                ymax = int(cy + h / 2)

                # chặn không cho vượt ra ngoài ảnh
                xmin = max(0, xmin)
                ymin = max(0, ymin)
                xmax = min(W - 1, xmax)
                ymax = min(H - 1, ymax)

                # dùng thẳng tâm cho servo, vẽ bbox và line 
                self.notify_servo_angle(int(cx), int(cy), W, H)
                
                cv2.line(cv_img, (W//2, H//2), (int(cx), int(cy)), (0, 255, 0), 3)
                cv2.rectangle(cv_img, (xmin, ymin), (xmax, ymax), (255, 0, 0), 4)

                self.get_logger().info(f"inference latency: {1000*(t2-t1):.0f}ms")
            else:
                self.get_logger().info(f"không tìm thấy vật thể")
            t3 = time.perf_counter()
            cv2.imshow('visualization', cv_img)
            cv2.waitKey(1)
            t4 = time.perf_counter()

            # đo được: preprocess~1ms | inference~14ms | postprocess~3ms | visualization~6ms  
            # tổng cả thread infer khoảng 22ms-30ms

            #self.get_logger().info(f"pre {1000*(t1-t0):.0f} | infer {1000*(t2-t1):.0f} | post {1000*(t3-t2):.0f} | show {1000*(t4-t3):.0f} ms")




def main(args=None):
    rclpy.init(args=args)
    try:
        node = inference_node()
        rclpy.spin(node)
    finally:
        node.cap.release() 
        node.destroy_node() 
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

# cần sửa-thêm: max_angle_one_side khi đổi cam, cần coi kĩ lại chỗ tính góc xoay khi dùng cam rời, hỏi thêm về preprocess