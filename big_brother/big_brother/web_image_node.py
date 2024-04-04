import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import time
import threading
from queue import Queue
from flask import Flask, Response, render_template_string

import cv2
from cv_bridge import CvBridge

app = Flask(__name__)
image_queue : Queue[Image] = Queue(maxsize=1)
sleep_duration = 0.0

class WebImageNode(Node):

    def __init__(self):
        super().__init__('web_image_node')

        self.declare_parameter('image_topic', 'camera_feed')
        self.declare_parameter('port', 5000)
        self.declare_parameter('update_rate', 30.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.port = self.get_parameter('port').value

        global sleep_duration
        sleep_duration = 1.0 / self.get_parameter('update_rate').value

        self.subscription = self.create_subscription(Image, self.image_topic, self.listener_callback, 10)

        self.bridge = CvBridge()

        self.get_logger().info(f'\n'
            f'\t\t---Web Image Node---\n'
            f'\t\t Image Topic: {self.image_topic}\n'
            f'\t\t Port: {self.port}\n'
            f'\t\t Update Rate: {self.get_parameter("update_rate").value}\n'
            f'\t\t-------------------\n'
        )

    def listener_callback(self, msg : Image):
        if not image_queue.empty():
            image_queue.get_nowait()
        image_queue.put(msg)

def gen_frames():
    while True:
        if image_queue.empty():
            time.sleep(sleep_duration)
            continue

        image = image_queue.get_nowait()
        
        cv_image = CvBridge().imgmsg_to_cv2(image, desired_encoding='bgr8')
        _, buffer = cv2.imencode('.png', cv_image)
        image_data = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/png\r\n\r\n' + image_data + b'\r\n')
        time.sleep(sleep_duration)

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template_string("""
    <html>
    <head>
        <style>
            body, html {
                margin: 0;
                padding: 0;
                overflow: hidden;
            }
            img {
                position: absolute;
                top: 0;
                left: 0;
                width: 100%;
                height: 100%;
            }
        </style>
    </head>
    <body>
        <img src="{{ url_for('video_feed') }}">
    </body>
    </html>
    """)

def main(args=None):

    rclpy.init(args=args)
    image_display_node = WebImageNode()

    port = image_display_node.port
    threading.Thread(target=app.run, kwargs={'host':'0.0.0.0', 'port':port}).start()

    rclpy.spin(image_display_node)

    image_display_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
