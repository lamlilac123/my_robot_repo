import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

import socket
from queue import Queue
from threading import Thread

# đây là IP cố đinh của esp32 vì tác giả đã cài IP tĩnh cho tiện
ESP32_IP = '192.168.0.245' # Đổi thành IP in ra trên Serial Monitor của ESP32
UDP_PORT = 4210

class SimpleSub(Node):
    def __init__(self):
        super().__init__('simple_sub')
        self.sub = self.create_subscription(Int32MultiArray, 'servo_angles', self.put_in_queue, 1)
        self.queue = Queue(maxsize=1)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.thread = Thread(target=self.binary_serial_writer, daemon=True)
        self.thread.start()



    def put_in_queue(self, msg):
        self.queue.put(msg)



    def binary_serial_writer(self):
        while rclpy.ok():
            # lấy pan tilt rối đóng gói thành binary và gửi đi
            msg = self.queue.get()

            pan = msg.data[0] & 0xFF
            tilt = msg.data[1] & 0xFF
            checksum = (0xAA ^ pan ^ tilt) & 0xFF

            packet = bytes([0xAA, pan, tilt, checksum])
            try:
                self.sock.sendto(packet, (ESP32_IP, UDP_PORT))
            except OSError as e:
                self.get_logger().warn(f'UDP send failed: {e}')



    def destroy_node(self):
        try:
            self.sock.close()
        except OSError:
            pass
        super().destroy_node()




def main():
    rclpy.init()
    node = SimpleSub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
