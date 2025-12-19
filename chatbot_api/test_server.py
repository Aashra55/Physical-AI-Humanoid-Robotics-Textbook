import http.server
import socketserver

PORT = 8000

# Simple handler that allows CORS
class CORSRequestHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        super().end_headers()

with socketserver.TCPServer(("", PORT), CORSRequestHandler) as httpd:
    print(f"Simple test server running at http://127.0.0.1:{PORT}")
    print("Press Ctrl+C to stop.")
    httpd.serve_forever()
