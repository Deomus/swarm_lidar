from mesh_interface.channel import DelayChannel
import pytest

def test_delay_preserves_measurement():
    channel = DelayChannel(delay=.2)
    message = dict(measurement_stamp=1.)
    channel.send(message, 1.)
    assert channel.receive(1.1) == []
    assert channel.receive(1.3) == [message]
    assert message['measurement_stamp'] == 1.

def test_loss_and_capacity():
    lost = DelayChannel(loss=1.)
    assert not lost.send('packet', 0.)
    bounded = DelayChannel(delay=1., capacity=1)
    assert bounded.send(1, 0.)
    assert not bounded.send(2, 0.)
    assert bounded.receive(2.) == [1]

def test_random_loss_is_reproducible():
    a, b = DelayChannel(loss=.5, seed=7), DelayChannel(loss=.5, seed=7)
    for i in range(100):
        a.send(i, 0.)
        b.send(i, 0.)
    assert a.receive(1.) == b.receive(1.)
    assert 0 < a.dropped < 100


def test_pose_loss_discards_queue_then_recovers_without_rewriting_time():
    poses, states, commands = DelayChannel(delay=.1), DelayChannel(), DelayChannel()
    poses.send('old', 1.)
    poses.drop_for(1.01, .7)
    assert poses.receive(1.2) == []
    assert not poses.send('lost', 1.6)
    for channel in (states, commands):
        assert channel.send('unaffected', 1.6)
        assert channel.receive(1.6) == ['unaffected']
    message = dict(stamp=1.8)
    assert poses.send(message, 1.8)
    assert poses.receive(2.) == [message]
    assert message['stamp'] == 1.8


@pytest.mark.parametrize('method', ['send', 'receive'])
def test_loss_window_and_queue_reset_on_clock_rewind(method):
    channel = DelayChannel()
    channel.drop_for(10., .7)
    if method == 'send':
        assert channel.send('new epoch', 1.)
    else:
        assert channel.receive(1.) == []
        assert channel.send('new epoch', 1.)
    assert channel.receive(1.) == ['new epoch']


@pytest.mark.parametrize('duration', [0., -1., float('nan'), float('inf')])
def test_invalid_loss_duration_preserves_queue(duration):
    channel = DelayChannel(delay=1.)
    channel.send('old', 1.)
    with pytest.raises(ValueError):
        channel.drop_for(1.1, duration)
    assert channel.receive(2.) == ['old']

