"""Клавиши, таймеры защиты и восстановление терминала без ROS."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from leader_interface.keyboard import KeyboardLogic, terminal_input  # noqa: E402


def ready(**parameters):
    logic = KeyboardLogic(**parameters)
    logic.note_clock(10., 100.)
    logic.note_state(10., True, 100.)
    logic.enabled = True
    return logic


@pytest.mark.parametrize('key,expected', [
    ('w', (.5, 0., 0., 0.)), ('s', (-.5, 0., 0., 0.)),
    ('a', (0., .5, 0., 0.)), ('d', (0., -.5, 0., 0.)),
    ('r', (0., 0., .5, 0.)), ('f', (0., 0., -.5, 0.)),
    ('q', (0., 0., 0., .3)), ('e', (0., 0., 0., -.3))])
def test_keys_have_enu_signs_and_bounded_velocity(key, expected):
    logic = ready()
    logic.key(key, 100.)
    assert logic.velocity(100.1) == expected


@pytest.mark.parametrize('preset,speed', [('1', .5), ('2', 1.), ('3', 1.5)])
def test_presets_and_no_combined_axis_amplification(preset, speed):
    logic = ready()
    logic.key(preset, 100.)
    logic.key('w', 100.)
    logic.key('a', 100.01)
    assert logic.velocity(100.02) == (0., speed, 0., 0.)


def test_deadman_requires_new_key_not_merely_new_clocks():
    logic = ready()
    logic.key('w', 100.)
    logic.note_clock(10.24, 100.24)
    logic.note_state(10.24, True, 100.24)
    assert logic.velocity(100.249)[0] == .5
    assert logic.velocity(100.25) == (0., 0., 0., 0.)
    logic.key('w', 100.26)
    assert logic.velocity(100.26)[0] == .5


def test_repeated_identical_clock_never_hides_pause():
    logic = ready()
    logic.key('w', 100.)
    logic.note_clock(10., 100.3)
    logic.note_state(10., True, 100.3)
    assert logic.problem(100.3) == 'CLOCK_STOPPED'
    assert logic.velocity(100.3) == (0., 0., 0., 0.)


def test_clock_rollback_requires_enter_and_fresh_direction_key():
    logic = ready()
    logic.key('w', 100.)
    logic.note_clock(2., 100.01)
    logic.note_state(2., True, 100.01)
    assert logic.problem(100.01) == 'CLOCK_JUMP'
    assert not logic.enabled
    assert logic.key('\n', 100.02) == 'MANUAL'
    assert logic.problem(100.02) is None
    assert logic.velocity(100.02) == (0., 0., 0., 0.)


@pytest.mark.parametrize('stamp', [11., float('nan'), -1.])
def test_clock_jump_or_invalid_clock_disables_old_command(stamp):
    logic = ready()
    logic.key('w', 100.)
    logic.note_clock(stamp, 100.01)
    assert not logic.enabled
    assert logic.velocity(100.01) == (0., 0., 0., 0.)


@pytest.mark.parametrize('stamp,valid,received,now', [(9., True, 100., 100.1),
                                                   (10.1, True, 100., 100.1),
                                                   (10., False, 100., 100.1),
                                                   (10., True, 99., 100.1),
                                                   (float('nan'), True, 100., 100.1)])
def test_invalid_future_or_old_state_blocks_motion(stamp, valid, received, now):
    logic = ready()
    logic.note_state(stamp, valid, received)
    assert logic.problem(now) == 'STATE_UNAVAILABLE'
    assert logic.velocity(now) == (0., 0., 0., 0.)


def test_hold_exit_and_resume_never_reuse_old_direction():
    logic = ready()
    logic.key('w', 100.)
    assert logic.key(' ', 100.01) == 'HOLD'
    assert not logic.enabled and logic.key_wall is None
    logic.key('w', 100.02)
    assert logic.key_wall is None
    assert logic.key('\r', 100.03) == 'MANUAL'
    assert not logic.enabled  # Включение только после ответа сервиса.
    assert logic.key('\x1b', 100.04) == 'EXIT'


def test_missing_data_never_enables_manual_and_custom_limits_are_respected():
    logic = KeyboardLogic(max_speed=.8, max_yaw_rate=.2)
    assert logic.key('\r', 1.) == 'HOLD'
    logic = ready(max_speed=.8, max_yaw_rate=.2)
    logic.key('3', 100.)
    logic.key('w', 100.)
    assert logic.velocity(100.1)[0] == .8
    logic.key('q', 100.1)
    assert logic.velocity(100.1)[3] == .2


@pytest.mark.parametrize('limits', [{'max_speed': 1.6}, {'max_yaw_rate': .31},
                                   {'deadman': 0}, {'clock_timeout': float('nan')}])
def test_unsafe_or_nonfinite_limits_are_rejected(limits):
    with pytest.raises(ValueError):
        KeyboardLogic(**limits)


def test_terminal_is_restored_after_exception(monkeypatch):
    import leader_interface.keyboard as keyboard

    saved = object()
    calls = []
    monkeypatch.setattr(keyboard.os, 'isatty', lambda descriptor: True)
    monkeypatch.setattr(keyboard.termios, 'tcgetattr', lambda descriptor: saved)
    monkeypatch.setattr(keyboard.tty, 'setcbreak', lambda descriptor: calls.append(('cbreak', descriptor)))
    monkeypatch.setattr(keyboard.termios, 'tcsetattr',
                        lambda descriptor, when, config: calls.append(('restore', descriptor, config)))
    class Stream:
        def fileno(self):
            return 12
    with pytest.raises(RuntimeError):
        with terminal_input(Stream()):
            raise RuntimeError('Проверка ошибки')
    assert calls == [('cbreak', 12), ('restore', 12, saved)]
