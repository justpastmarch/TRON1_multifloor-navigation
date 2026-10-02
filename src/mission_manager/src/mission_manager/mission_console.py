"""Loopback web console hosted by the existing mission-manager process.
No ROS node initialization, velocity publisher, robot socket, or shell execution.
"""
import json
import secrets
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse
from .console_door import ConsoleDoor


class ConsoleServer:
    def __init__(self, backend, html_path, port=8765, *, door_factory=ConsoleDoor):
        self.backend = backend
        self.html = Path(html_path).read_bytes()
        self.token = secrets.token_urlsafe(32)
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def reply(self, code, value, kind='application/json; charset=utf-8'):
                data = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False).encode()
                self.send_response(code)
                self.send_header('Content-Type', kind)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.end_headers()
                self.wfile.write(data)

            def trusted_host(self):
                return self.headers.get('Host') in ('127.0.0.1:%d' % owner.http.server_port, 'localhost:%d' % owner.http.server_port)

            def do_GET(self):
                if not self.trusted_host():
                    return self.reply(403, {'error': 'invalid host'})
                path = urlparse(self.path).path
                if path == '/':
                    return self.reply(200, owner.html, 'text/html; charset=utf-8')
                if path == '/api/state':
                    result = owner.backend.snapshot()
                    result['door_control'] = owner.door.snapshot()
                    result['door'] = result['door_control']['reason']
                    result['token'] = owner.token
                    return self.reply(200, result)
                if path == '/api/map':
                    return self.reply(200, owner.backend.map_snapshot())
                return self.reply(404, {'error': 'not found'})

            def do_POST(self):
                if not self.trusted_host():
                    return self.reply(403, {'error': 'invalid host'})
                origin = self.headers.get('Origin')
                expected = 'http://' + self.headers.get('Host', '')
                if (origin is not None and origin != expected) or not secrets.compare_digest(self.headers.get('X-TRON-Token', ''), owner.token):
                    return self.reply(403, {'error': '이 화면을 새로 열고 다시 시도하세요.'})
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 16384:
                        raise ValueError('invalid request size')
                    data = json.loads(self.rfile.read(length))
                    if not isinstance(data, dict):
                        raise ValueError('request must be an object')
                    if self.path == '/api/operator_phase':
                        return self.reply(200, owner.backend.operator_phase(data))
                    if self.path == '/api/door/open_once':
                        return self.reply(200, owner.door.manual_open(data))
                    if self.path == '/api/photo/roof_start':
                        return self.reply(200, owner.backend.start_rooftop_photos(data))
                    if self.path == '/api/photo/capture_mail':
                        return self.reply(200, owner.backend.capture_and_mail(data))
                    if self.path == '/api/mail/settings':
                        return self.reply(200, owner.backend.mail_settings(data))
                    if self.path == '/api/mail/retry':
                        return self.reply(200, owner.backend.mail_retry(data))
                    if self.path.startswith('/api/manual/'):
                        operation = self.path.rsplit('/', 1)[-1]
                        if operation not in ('begin', 'update', 'end'):
                            return self.reply(404, {'error':'not found'})
                        return self.reply(200, getattr(owner.backend.manual, operation)(data))
                    if self.path == '/api/return':
                        return self.reply(200, owner.backend.approve_return(data))
                    if self.path == '/api/recover_control':
                        return self.reply(200, owner.backend.recover_control(data))
                    if self.path == '/api/select_floor':
                        return self.reply(200, owner.backend.select_floor(data))
                    method = {'/api/door/return': owner.door.manual_return,
                              '/api/door/stop': owner.door.stop,
                              '/api/plan': owner.backend.plan,
                              '/api/start': owner.backend.start,
                              '/api/cancel': owner.backend.cancel}.get(self.path)
                    if method is None:
                        return self.reply(404, {'error': 'not found'})
                    return self.reply(200, method(data))
                except (ValueError, KeyError, TypeError) as error:
                    return self.reply(409, {'error': str(error)})
                except Exception:
                    return self.reply(500, {'error': '요청 처리 실패. ROS 로그를 확인하세요.'})

        self.http = ThreadingHTTPServer(('127.0.0.1', port), Handler)
        try:
            self.door = door_factory(backend.snapshot)
        except Exception:
            self.http.server_close()
            raise
        self.thread = threading.Thread(target=self.http.serve_forever, name='mission-console-http', daemon=True)
        self.thread.start()

    def close(self):
        self.door.close()
        self.http.shutdown()
        self.http.server_close()
        self.thread.join(timeout=2)
