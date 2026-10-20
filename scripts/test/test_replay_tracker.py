"""Ограничения воспроизведения проверяются без ROS и внешних процессов."""

import importlib.util
from pathlib import Path
import signal
import subprocess
from types import SimpleNamespace

import pytest
import yaml


@pytest.fixture
def replay():
    spec = importlib.util.spec_from_file_location('replay_under_test',
                                                Path(__file__).parents[1] / 'replay_tracker.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def metadata(replay, directory, **changes):
    info = {'storage_identifier': 'mcap', 'starting_time': {'nanoseconds_since_epoch': 52_000_000},
            'duration': {'nanoseconds': 225_400_000_000},
            'topics_with_message_count': [
                {'topic_metadata': {'name': name, 'type': kind}, 'message_count': 5}
                for name, kind in replay.REQUIRED_TYPES.items()]}
    info.update(changes)
    (directory / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': info}))


def test_metadata_requires_cloud_and_ground_truth(replay, tmp_path):
    metadata(replay, tmp_path)
    start, end, topics = replay.inspect_bag(tmp_path)
    assert start == pytest.approx(.052)
    assert end == pytest.approx(225.452)
    assert topics == [replay.CLOUD, replay.TRUTH]
    metadata(replay, tmp_path, topics_with_message_count=[])
    with pytest.raises(ValueError, match='Нет непустой'):
        replay.inspect_bag(tmp_path)


@pytest.mark.parametrize('count,kind', [(0, 'sensor_msgs/msg/PointCloud2'),
                                       (1, 'std_msgs/msg/String')])
def test_empty_or_wrong_cloud_rejected(replay, tmp_path, count, kind):
    metadata(replay, tmp_path)
    path = tmp_path / 'metadata.yaml'
    info = yaml.safe_load(path.read_text())
    row = info['rosbag2_bagfile_information']['topics_with_message_count'][0]
    row['message_count'] = count
    row['topic_metadata']['type'] = kind
    path.write_text(yaml.safe_dump(info))
    with pytest.raises(ValueError, match='Нет непустой'):
        replay.inspect_bag(tmp_path)


def test_playback_never_includes_old_estimates_commands_or_recorded_clock(replay):
    command = replay.play_command(Path('/record'), 49., 60., .052, .5, replay.REPLAY_TOPICS)
    topics = command[command.index('--topics') + 1:]
    assert replay.CLOUD in topics and replay.TRUTH in topics
    assert not any('relative_poses' in topic or 'commands' in topic for topic in topics)
    assert '/clock' not in topics
    assert command[command.index('--start-offset') + 1] == str(49. - .052)
    assert command[command.index('--playback-duration') + 1] == '11.0'


def test_forbidden_topic_cannot_be_added(replay):
    with pytest.raises(ValueError, match='разрешённого'):
        replay.play_command(Path('/record'), 49., 60., .052, .5, ['/swarm/relative_poses'])


def test_reliable_playback_explicitly_overrides_publisher_qos(replay):
    command = replay.play_command(Path('/record'), 49., 60., .052, .5,
                                 replay.REPLAY_TOPICS, Path('/output/qos.yaml'))
    index = command.index('--qos-profile-overrides-path')
    assert command[index + 1] == '/output/qos.yaml'
    assert command.index('--topics') > index + 1


@pytest.mark.parametrize('domain,current', [(72, 0), (83, 83), (-1, 0), (102, 0)])
def test_replay_refuses_live_or_unsupported_domains(replay, domain, current):
    with pytest.raises(ValueError, match='ROS_DOMAIN_ID'):
        replay.validate_window(49., 60., .052, 225., .5, domain, current)


@pytest.mark.parametrize('start,end,rate', [(49., 60., float('nan')), (49., 60., 0.),
                                          (49., 60., 1.1), (0., 60., .5), (60., 49., .5),
                                          (49., 226., .5), (float('inf'), 60., .5)])
def test_invalid_windows_and_rates(replay, start, end, rate):
    with pytest.raises(ValueError):
        replay.validate_window(start, end, .052, 225., rate, 83, 72)


def test_own_process_escalates_only_its_group(replay, monkeypatch):
    sent = []
    monkeypatch.setattr(replay.os, 'killpg', lambda pid, signum: sent.append((pid, signum)))
    waits = []
    def wait(timeout):
        waits.append(timeout)
        if len(waits) < 3:
            raise subprocess.TimeoutExpired('owned', timeout)
    process = SimpleNamespace(pid=12345, poll=lambda: None, wait=wait)
    replay.stop_owned(process)
    assert sent == [(12345, signal.SIGINT), (12345, signal.SIGTERM), (12345, signal.SIGKILL)]
    assert waits == [10, 5, 5]


def test_finished_process_never_signalled(replay, monkeypatch):
    monkeypatch.setattr(replay.os, 'killpg', lambda *args: pytest.fail('Завершённый процесс не трогать'))
    replay.stop_owned(SimpleNamespace(poll=lambda: 0))


def test_ready_requires_tracker_and_recorder_subscriptions(replay):
    spins = []
    executor = SimpleNamespace(spin_once=lambda **kwargs: spins.append(kwargs))
    node = SimpleNamespace(count_subscribers=lambda topic: 2 if topic.endswith('relative_poses') else 1,
                           count_publishers=lambda topic: 1)
    active = SimpleNamespace(poll=lambda: None)
    replay.wait_ready(node, executor, active, active, replay.time.monotonic() + 1)
    assert not spins


def test_dead_worker_stops_before_playback(replay):
    dead = SimpleNamespace(poll=lambda: 1, returncode=1)
    with pytest.raises(RuntimeError, match='до воспроизведения'):
        replay.wait_ready(None, None, dead, dead, replay.time.monotonic() + 1)
