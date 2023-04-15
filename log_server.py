import http.server
import socketserver
import datetime
import json
import os

PORT = 8000


class Handler(http.server.BaseHTTPRequestHandler):
    def _set_headers(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()
    
    def do_GET(self):
        self._set_headers()
    
    def do_HEAD(self):
        self._set_headers()

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        self._set_headers()
        print(post_data)
        data = json.loads(post_data)
        hostname = os.path.basename(os.path.normpath(data['host']))
        with open(f"{hostname}_sensor.log", 'a+') as log:
            timestamp = datetime.datetime.now().isoformat()
            log.write(f"{timestamp}\t{data['reading']}\n")
        

handler = http.server.SimpleHTTPRequestHandler

with socketserver.TCPServer(("0.0.0.0", PORT), Handler) as httpd:
    print(f"Serving at port {PORT}")
    httpd.serve_forever()