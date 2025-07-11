# robocar_server_rtsp.py

import threading
import subprocess
import time
from flask import Flask, jsonify
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

app = Flask(__name__)
odom_cache = {"data": None}
lock = threading.Lock()

# 1) RTSP 서버 기동 (gst-rtsp-launch 필요)
def start_rtsp_server():
    # 해상도·프레임·비트레이트는 필요에 맞게 조정
    width, height, framerate, bitrate = 1280, 720, 30, 4000000
    pipeline = (
        f"( nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM),width={width},height={height},framerate={framerate}/1 ! "
        f"nvvidconv ! "
        f"omxh264enc bitrate={bitrate} ! "
        f"h264parse ! "
        f"rtph264pay name=pay0 pt=96 )"
    )
    # gst-rtsp-launch 가 PATH에 있어야 함
    subprocess.Popen(['gst-rtsp-launch', pipeline])

# 2) ROS2 오도메트리 구독
class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_bridge')
        self.create_subscription(Odometry, '/wheel/odom',
                                 self.cb, 10)
    def cb(self, msg):
        data = {
            "sec": msg.header.stamp.sec,
            "nanosec": msg.header.stamp.nanosec,
            "pose": {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
            },
            "twist": {
                "linear_x": msg.twist.twist.linear.x
            }
        }
        with lock:
            odom_cache["data"] = data

def ros_thread():
    rclpy.init()
    node = OdomNode()
    rclpy.spin(node)

# 3) Flask 오도메트리 라우트
@app.route('/odometry')
def get_odom():
    with lock:
        d = odom_cache["data"]
    if d is None:
        return jsonify({"error": "no odom"}), 204
    return jsonify(d), 200

if __name__ == "__main__":
    # RTSP 서버
    threading.Thread(target=start_rtsp_server, daemon=True).start()
    time.sleep(1)  # 서버 기동 대기
    # ROS 스핀
    threading.Thread(target=ros_thread, daemon=True).start()
    # Flask 시작 (영상 스트리밍은 RTSP로 대체)
    app.run(host='0.0.0.0', port=8000)
