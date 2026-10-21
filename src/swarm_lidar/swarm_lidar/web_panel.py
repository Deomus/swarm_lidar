"""Локальная браузерная панель симуляции без внешнего HTTP-фреймворка."""

import argparse
import fcntl
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import secrets
import signal
import threading
from urllib.parse import urlsplit


def make_server(backend, assets, port):
    token = secrets.token_urlsafe(32)
    assets = Path(assets)

    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(5.)

        def log_message(self, *args):
            pass

        def send(self, code, body, mime='application/json; charset=utf-8'):
            data = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False,
                                                                  allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; object-src 'none'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(data)

        def allowed(self):
            actual_port = self.server.server_port
            allowed_hosts = (f'localhost:{actual_port}', f'127.0.0.1:{actual_port}')
            if self.headers.get('Host') not in allowed_hosts:
                self.send(403, dict(error='Недопустимый адрес панели'))
                return False
            origin = self.headers.get('Origin')
            if origin is not None and origin not in tuple('http://'+host for host in allowed_hosts):
                self.send(403, dict(error='Запрос должен исходить из локальной панели'))
                return False
            return True

        def do_GET(self):
            if not self.allowed():
                return
            path = urlsplit(self.path).path
            if path == '/api/state':
                self.send(200, backend.snapshot())
            elif path == '/':
                html = (assets/'index.html').read_text().replace('__PANEL_TOKEN__', token)
                self.send(200, html.encode(), 'text/html; charset=utf-8')
            elif path in ('/app.js', '/style.css'):
                mime = 'text/javascript' if path.endswith('.js') else 'text/css'
                self.send(200, (assets/path[1:]).read_bytes(), mime+'; charset=utf-8')
            else:
                self.send(404, dict(error='Страница не найдена'))

        def do_POST(self):
            if not self.allowed():
                return
            if self.path != '/api/command':
                self.send(404, dict(error='Команда не найдена'))
                return
            if not secrets.compare_digest(self.headers.get('X-Panel-Token', ''), token):
                self.send(403, dict(error='Обновите страницу панели'))
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 4096:
                    raise ValueError('Недопустимый размер запроса')
                if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    raise ValueError('Ожидается JSON')
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict) or not isinstance(body.get('action'), str):
                    raise ValueError('Некорректная команда')
                backend.submit(body['action'], body)
                self.send(202, dict(accepted=True))
            except (ValueError, TypeError) as error:
                self.send(400, dict(error=str(error)))
            except Exception:
                self.send(500, dict(error='Команда не выполнена; проверьте терминал панели'))

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--log-dir', type=Path, default=Path('log/web'))
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error('Порт должен быть в диапазоне 1024–65535')
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from rclpy.signals import SignalHandlerOptions
    from .web_ros import PanelBackend

    # Одна панель на домен ROS; разные порты не должны создавать двух операторов.
    lock_path = Path('/tmp')/f'swarm_lidar_panel_{os.getuid()}_{int(os.getenv("ROS_DOMAIN_ID", "0"))}.lock'
    lock = lock_path.open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('Панель этого домена ROS уже запущена')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    backend = PanelBackend(args.log_dir)
    assets = Path(get_package_share_directory('swarm_lidar'))/'web'
    try:
        server = make_server(backend, assets, args.port)
    except OSError as error:
        backend.destroy_node()
        rclpy.shutdown()
        parser.error(str(error))
    stop = threading.Event()

    def spin():
        while not stop.is_set():
            rclpy.spin_once(backend, timeout_sec=.1)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    print(f'Панель симуляции: http://localhost:{args.port}', flush=True)
    print('Для завершения — Ctrl+C. При активном полёте сначала выполняется посадка.', flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        with backend.lock:
            backend.closing = True
        server.server_close()
        if backend.flight.active():
            print('Ожидание посадки аппаратов…', flush=True)
        landed = backend.flight.close()
        stop.set()
        thread.join(timeout=2.)
        backend.destroy_node()
        rclpy.shutdown()
        lock.close()
        if not landed:
            raise RuntimeError('Посадка не подтверждена вовремя; исполнитель оставлен работающим. '
                               'Не останавливайте симуляцию, проверьте журнал полёта.')


if __name__ == '__main__':
    main()
