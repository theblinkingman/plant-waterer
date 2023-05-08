import datetime
import json
import os
from waitress import serve

PORT = 8000


def handle_POST(environ):
    content_length = int(environ.get('CONTENT_LENGTH', 0))
    post_data = environ['wsgi.input'].read(content_length)
    print(post_data)
    data = json.loads(post_data)
    hostname = os.path.basename(os.path.normpath(data['host']))
    with open(f"{hostname}_sensor.log", 'a+') as log:
        timestamp = datetime.datetime.now().isoformat()
        log.write(f"{timestamp}\t{data['reading']}\n")

def wsgiapp(environ, start_fn):

    if environ['REQUEST_METHOD'] == "POST":
        handle_POST(environ)

    body = b'Hello world!\n'
    status = '200 OK'
    headers = [('Content-type', 'text/plain')]
    start_fn(status, headers)
    return [body]
    
print(f"Server starting on port {PORT}")
serve(wsgiapp, listen=f"*:{PORT}")
