# robocar_server_rtsp_pybind.py

import threading
import time
from flask import Flask, jsonify
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# GObject/GstRTSPServer 바인딩
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import GObject, Gst, GstRtspServer

Gst.init(None)

app = Flask(__name__)
odom_cache = {'data': None}
lock = threading.Lock()

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
            odom_cache['data'] = data

@app.route('/odometry')
def get_odom():
    with lock:
        d = odom_cache['data']
    if d is None:
        return jsonify({"error": "no odom"}), 204
    return jsonify(d), 200

def ros_thread():
    rclpy.init()
    node = OdomNode()
    rclpy.spin(node)

def rtsp_thread():
    class SensorFactory(GstRtspServer.RTSPMediaFactory):
        def __init__(self):
            super().__init__()
            self.set_shared(True)
        def do_create_element(self, url):
            pipeline_str = (
                "nvarguscamerasrc ! "
                "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
                "nvvidconv ! "
                "omxh264enc bitrate=4000000 control-rate=1 ! "
                "h264parse ! rtph264pay name=pay0 pt=96"
            )
            return Gst.parse_launch(pipeline_str)

    server = GstRtspServer.RTSPServer()
    mounts = server.get_mount_points()
    mounts.add_factory("/test", SensorFactory())
    server.attach(None)
    loop = GObject.MainLoop()
    loop.run()

if __name__ == '__main__':
    # 1) RTSP 서버 (Python 바인딩)
    threading.Thread(target=rtsp_thread, daemon=True).start()
    time.sleep(1)
    # 2) ROS2 오도메트리
    threading.Thread(target=ros_thread, daemon=True).start()
    # 3) Flask 오도메트리 서비스
    app.run(host='0.0.0.0', port=8000)
