#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading

class WebCameraNode(DTROS):
    """Minimal web-based camera viewer at http://localhost:8080"""

    def __init__(self, node_name):
        super(WebCameraNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'deutschbot')
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._latest_jpeg = None
        
        # Start web server
        self._start_server()
        
        # Subscribe to camera
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self._on_image, queue_size=1)
        
        self.log(f"📺 Camera viewer: http://localhost:8080")

    def _start_server(self):
        viewer = self
        
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/':
                    self.send_response(200)
                    self.send_header('Content-type', 'text/html')
                    self.end_headers()
                    self.wfile.write(b"""<!DOCTYPE html>
<html>
<head>
    <title>Camera</title>
    <style>
        body { margin: 0; background: #000; display: flex; justify-content: center; align-items: center; height: 100vh; }
        img { max-width: 95vw; max-height: 95vh; image-rendering: -webkit-optimize-contrast; }
    </style>
</head>
<body>
    <img id="cam" src="/image">
    <script>
        setInterval(() => { document.getElementById('cam').src = '/image?' + Date.now(); }, 33);
    </script>
</body>
</html>""")
                elif self.path.startswith('/image'):
                    if viewer._latest_jpeg:
                        self.send_response(200)
                        self.send_header('Content-type', 'image/jpeg')
                        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                        self.end_headers()
                        self.wfile.write(viewer._latest_jpeg)
                    else:
                        self.send_response(503)
                        self.end_headers()
                else:
                    self.send_response(404)
                    self.end_headers()
            
            def log_message(self, format, *args):
                pass
        
        server = HTTPServer(('0.0.0.0', 8080), Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()

    def _on_image(self, msg):
        self._latest_jpeg = bytes(msg.data)

if __name__ == '__main__':
    node = WebCameraNode(node_name='web_camera_node')
    rospy.spin()
