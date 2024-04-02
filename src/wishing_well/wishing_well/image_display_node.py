import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import threading
import requests
import queue
from flask import Flask, Response, render_template_string

app = Flask(__name__)
image_url_queue = queue.Queue(maxsize=1)

class ImageDisplayNode(Node):
    
        def __init__(self):
            super().__init__('image_display_node')

            self.subscription = self.create_subscription(String, 'dalle_image_url', self.listener_callback, 10)

            self.get_logger().info(f'Image Display Node Initialized')
    
        def listener_callback(self, msg : String):
            self.get_logger().info(f'Updating Image...')
            if not image_url_queue.empty():
              image_url_queue.get_nowait()
            image_url_queue.put(msg.data)
            self.get_logger().info(f'Updated Image: {msg.data}')

def gen_frames():
    while True:

      image_url = image_url_queue.get_nowait()
      image_url_queue.put(image_url)

      response = requests.get(image_url)
      if response.status_code == 200:
          image_data = response.content 
          yield (b'--frame\r\n'
                  b'Content-Type: image/png\r\n\r\n' + image_data + b'\r\n')
      else:
          print(f"Failed to fetch image from {image_url}, status code: {response.status_code}")

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
    image_display_node = ImageDisplayNode()
    rclpy.spin(image_display_node)
    image_display_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    threading.Thread(target=main).start()
    app.run(host='0.0.0.0', port=5000)