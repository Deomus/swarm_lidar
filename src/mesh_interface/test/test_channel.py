from mesh_interface.channel import DelayChannel

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

