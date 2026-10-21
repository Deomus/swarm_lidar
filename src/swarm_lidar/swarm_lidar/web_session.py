"""Один принадлежащий панели полётный процесс и ограниченный журнал событий."""

from collections import deque
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading


class FlightSession:
    def __init__(self, log_dir):
        self.log_dir = Path(log_dir)
        self.lock = threading.RLock()
        self.process = None
        self.thread = None
        self.phase = 'idle'
        self.kind = None
        self.output = None
        self.cancelled = False
        self.logs = deque(maxlen=80)

    def active(self):
        return self.phase in ('starting', 'running', 'ready', 'landing')

    def snapshot(self):
        with self.lock:
            return dict(phase=self.phase, kind=self.kind,
                        output=str(self.output) if self.output else None, logs=list(self.logs))

    def start(self, kind, record=True):
        if kind not in ('operator', 's1', 's2', 's3') or not isinstance(record, bool):
            raise ValueError('Неизвестный сценарий или параметр записи')
        with self.lock:
            if self.active():
                raise ValueError('Сначала завершите текущий полёт')
            self.log_dir.mkdir(parents=True, exist_ok=True)
            self.output = Path(tempfile.mkdtemp(prefix=kind+'_', dir=self.log_dir.resolve()))
            # Исполнитель сам создаёт каталог результатов и не перезаписывает существующий.
            self.output = self.output/'flight'
            args = [sys.executable, '-m', 'swarm_lidar.scenario', kind,
                    '--output', str(self.output)]
            if record:
                args.append('--record-bag')
            self.phase, self.kind, self.cancelled = 'starting', kind, False
            self.logs.clear()
            try:
                self.process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                                text=True, bufsize=1, start_new_session=True)
            except Exception:
                self.phase = 'error'
                raise
            self.thread = threading.Thread(target=self._read, daemon=True)
            self.thread.start()

    def _read(self):
        with (self.output.parent/'console.log').open('w') as log:
            for line in self.process.stdout:
                log.write(line)
                log.flush()
                with self.lock:
                    self.logs.append(line.rstrip()[:600])
                    if self.phase != 'landing':
                        if 'Ожидание оператора' in line:
                            self.phase = 'ready'
                        elif self.phase == 'starting' and 'PX4 готовы' in line:
                            self.phase = 'running'
        self.process.wait()
        self.process.stdout.close()
        try:
            result = json.loads((self.output/'result.json').read_text())
        except (OSError, ValueError):
            result = {}
        with self.lock:
            if self.cancelled and result.get('passed_landing'):
                self.phase = 'cancelled'
            elif self.process.returncode == 0 and result.get('passed'):
                self.phase = 'complete'
            else:
                self.phase = 'error'
                self.logs.append('Полёт не завершён штатно: проверьте журнал и подтверждение посадки.')

    def mark_landing(self):
        with self.lock:
            if self.active():
                self.phase = 'landing'

    def stop(self):
        with self.lock:
            if self.active() and self.process is not None and self.process.poll() is None:
                self.phase = 'landing'
                if not self.cancelled:
                    self.cancelled = True
                    # Исполнитель ловит SIGINT и выполняет посадку в finally.
                    self.process.send_signal(signal.SIGINT)

    def close(self, timeout=100.):
        self.stop()
        if self.thread:
            self.thread.join(timeout=timeout)
        return self.thread is None or not self.thread.is_alive()
