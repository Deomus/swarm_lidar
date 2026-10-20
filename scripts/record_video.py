#!/usr/bin/env python3
"""Запись реальных RGB-кадров Gazebo в новый файл MP4, без ROS-моста."""

import argparse
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread

WIDTH, HEIGHT, FPS = 1280, 720, 10


def decode_rgb(message, width=WIDTH, height=HEIGHT):
    """Проверяет RGB_INT8 и убирает дополнение в конце строк изображения."""
    if message.pixel_format_type != 3:
        raise ValueError('Ожидается RGB_INT8, код формата 3')
    if message.width != width or message.height != height:
        raise ValueError(f'Ожидается изображение {width}x{height}')
    row_bytes = width * 3
    if message.step < row_bytes:
        raise ValueError('Длина строки меньше ширины RGB-изображения')
    if len(message.data) != message.step * height:
        raise ValueError('Размер данных не соответствует высоте и длине строки')
    if message.step == row_bytes:
        return bytes(message.data)
    return b''.join(message.data[y * message.step:y * message.step + row_bytes]
                    for y in range(height))


def resolve_ffmpeg(explicit):
    """Находит явно заданный исполняемый файл либо ffmpeg в PATH."""
    executable = str(Path(explicit).expanduser()) if explicit else shutil.which('ffmpeg')
    if not executable or not Path(executable).is_file() or not os.access(executable, os.X_OK):
        raise ValueError('Не найден ffmpeg: установите его или передайте --ffmpeg /путь/к/ffmpeg')
    return executable


def record(args):
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    executable = resolve_ffmpeg(args.ffmpeg)
    output = Path(args.output).expanduser()
    # Режим xb резервирует только новый файл, существующий никогда не затирается.
    with output.open('xb') as handle:
        command = [executable, '-hide_banner', '-loglevel', 'error', '-f', 'rawvideo',
                   '-pixel_format', 'rgb24', '-video_size', f'{WIDTH}x{HEIGHT}',
                   '-framerate', str(FPS), '-i', 'pipe:0', '-an', '-c:v', 'libx264',
                   '-preset', 'veryfast', '-crf', '23', '-pix_fmt', 'yuv420p',
                   '-movflags', '+frag_keyframe+empty_moov+default_base_moof',
                   '-f', 'mp4', 'pipe:1']
        # Ctrl+C получает записывающая программа; кодировщик завершается
        # после закрытия входа, успевая дописать контейнер без второго SIGINT.
        encoder = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=handle,
                                   start_new_session=True)
        frames = Queue(maxsize=5)
        stopping, first_frame = Event(), Event()
        lock = Lock()
        counts = {'received': 0, 'written': 0, 'dropped': 0, 'invalid': 0}
        errors = []
        write_failed = Event()
        follow_lock = Lock()
        follow_center = None
        follow_stamp = None
        follow_counts = {'moved': 0, 'failed': 0}
        pose_topic = '/world/swarm/pose/info'

        def receive_poses(message):
            nonlocal follow_center, follow_stamp
            if stopping.is_set():
                return
            positions = {}
            for pose in message.pose:
                if pose.name in ('uav_0', 'uav_1', 'uav_2', 'uav_3'):
                    position = (pose.position.x, pose.position.y, pose.position.z)
                    if all(math.isfinite(value) for value in position):
                        positions[pose.name] = position
            if len(positions) != 4:
                return
            center = tuple(sum(position[axis] / 4 for position in positions.values())
                           for axis in range(3))
            received = time.monotonic()
            with follow_lock:
                follow_center, follow_stamp = center, received

        def move_camera(now):
            with follow_lock:
                center, stamp = follow_center, follow_stamp
            if center is None or stamp is None or not 0 <= now - stamp <= 1.:
                return
            pose = Pose(name='spectator_camera')
            pose.position.x = center[0]
            pose.position.y = center[1] - 16.
            pose.position.z = center[2] + 17.
            # Поворот вокруг Z, затем наклон вокруг Y: оптическая ось +X
            # направлена к центру группы из положения над ней и сбоку.
            half_pitch = math.atan2(17., 16.) / 2
            half_yaw = math.pi / 4
            pose.orientation.x = -math.sin(half_pitch) * math.sin(half_yaw)
            pose.orientation.y = math.sin(half_pitch) * math.cos(half_yaw)
            pose.orientation.z = math.cos(half_pitch) * math.sin(half_yaw)
            pose.orientation.w = math.cos(half_pitch) * math.cos(half_yaw)
            try:
                completed, response = transport.request(
                    '/world/swarm/set_pose', pose, Pose, Boolean, 100)
                accepted = bool(completed and response is not None and response.data)
            except (RuntimeError, ValueError) as error:
                accepted = False
                detail = str(error)
            else:
                detail = 'таймаут или отрицательный ответ Gazebo'
            if accepted:
                follow_counts['moved'] += 1
            else:
                follow_counts['failed'] += 1
                if follow_counts['failed'] == 1 or follow_counts['failed'] % 10 == 0:
                    print(f'Предупреждение: камера не перемещена ({detail}); '
                          f'неудачных запросов {follow_counts["failed"]}.',
                          file=sys.stderr, flush=True)

        def receive(message):
            with lock:
                if stopping.is_set():
                    return
                try:
                    frame = decode_rgb(message)
                except ValueError as error:
                    counts['invalid'] += 1
                    if not errors:
                        errors.append(str(error))
                    return
                counts['received'] += 1
                first_frame.set()
                if frames.full():
                    try:
                        frames.get_nowait()
                        counts['dropped'] += 1
                    except Empty:
                        pass
                try:
                    frames.put_nowait(frame)
                except Full:
                    counts['dropped'] += 1

        def write_frames():
            try:
                while not stopping.is_set() or not frames.empty():
                    try:
                        frame = frames.get(timeout=.1)
                    except Empty:
                        continue
                    encoder.stdin.write(frame)
                    counts['written'] += 1
            except (BrokenPipeError, OSError) as error:
                errors.append(f'Запись в ffmpeg остановлена: {error}')
                write_failed.set()
                stopping.set()
            finally:
                try:
                    encoder.stdin.close()
                except OSError:
                    pass

        transport = Node()
        writer = Thread(target=write_frames, name='gazebo_video_writer', daemon=True)
        writer.start()
        started = time.monotonic()
        next_follow = started
        interrupted = False
        try:
            transport.subscribe(Image, args.topic, receive)
            if args.follow:
                transport.subscribe(Pose_V, pose_topic, receive_poses)
            print(f'Запись {args.topic} → {output}; остановка Ctrl+C', flush=True)
            while not stopping.wait(.1):
                now = time.monotonic()
                elapsed = now - started
                if encoder.poll() is not None:
                    raise RuntimeError(f'ffmpeg завершился с кодом {encoder.returncode}')
                if not first_frame.is_set() and elapsed >= args.startup_timeout:
                    detail = errors[0] if errors else 'кадры не поступают'
                    raise RuntimeError(f'Нет пригодного изображения: {detail}')
                if args.duration and elapsed >= args.duration:
                    break
                if args.follow and now >= next_follow:
                    next_follow = now + .2
                    move_camera(now)
        except KeyboardInterrupt:
            interrupted = True
        finally:
            previous_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
            with lock:
                stopping.set()
            transport.unsubscribe(args.topic)
            if args.follow:
                transport.unsubscribe(pose_topic)
            writer.join(timeout=10.)
            if writer.is_alive():
                encoder.terminate()
                writer.join(timeout=2.)
            try:
                code = encoder.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                encoder.kill()
                code = encoder.wait(timeout=5.)
            finally:
                signal.signal(signal.SIGINT, previous_sigint)
        print(f'Кадров: получено {counts["received"]}, записано {counts["written"]}, '
              f'пропущено {counts["dropped"]}, некорректно {counts["invalid"]}.', flush=True)
        if args.follow:
            print(f'Камера: подтверждено перемещений {follow_counts["moved"]}, '
                  f'неудачных запросов {follow_counts["failed"]}.', flush=True)
            if not follow_counts['moved']:
                print('Предупреждение: сопровождение группы не подтверждено; '
                      'видео могло остаться со статичной камеры.', file=sys.stderr, flush=True)
        if code != 0 or not counts['written'] or write_failed.is_set():
            detail = errors[-1] if errors else (
                'нет записанных кадров' if not counts['written'] else 'кодировщик завершился с ошибкой')
            raise RuntimeError(f'Видео не завершено успешно (код {code}): {detail}')
        reason = 'по Ctrl+C' if interrupted else 'по заданной длительности'
        print(f'Сохранено {output} ({reason}), длительность видео '
              f'{counts["written"] / FPS:.1f} с при {FPS} кадрах/с.', flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', help='Новый файл MP4; родительский каталог должен существовать')
    parser.add_argument('--duration', type=float, default=0.,
                        help='Длительность записи по реальным часам, секунды; 0 — до Ctrl+C')
    parser.add_argument('--topic', default='/evaluation/video', help='Тема изображения Gazebo')
    parser.add_argument('--follow', action='store_true',
                        help='Перемещать обзорную камеру вслед за центром четырёх аппаратов')
    parser.add_argument('--ffmpeg', help='Путь к исполняемому файлу ffmpeg; иначе поиск в PATH')
    parser.add_argument('--startup-timeout', type=float, default=30.,
                        help='Ожидание первого пригодного кадра, реальные секунды')
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration должен быть конечным неотрицательным числом')
    if not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0:
        parser.error('--startup-timeout должен быть положительным конечным числом')
    try:
        record(args)
    except (ImportError, OSError, ValueError, RuntimeError) as error:
        print(f'Ошибка записи: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
