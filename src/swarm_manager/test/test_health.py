from swarm_manager.health import data_status

def test_stale_transitions():
    assert data_status(1., .9, True) == 'FOLLOWING'
    assert data_status(1., .5, True) == 'DEGRADED'
    assert data_status(2., .5, True) == 'HOLD'

def test_invalid_missing_and_future():
    assert data_status(1., .9, False) == 'HOLD'
    assert data_status(1., None, True) == 'HOLD'
    assert data_status(1., 2., True) == 'HOLD'


def test_manager_recovers_new_epoch_and_ignores_future(monkeypatch):
    from types import SimpleNamespace
    import rclpy
    from swarm_interfaces.msg import SwarmState
    from swarm_manager.node import SwarmManager
    rclpy.init()
    node = SwarmManager()
    clock = [100.]
    monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=round(clock[0]*1e9))))
    try:
        for stamp in [100, 1]:
            clock[0] = float(stamp)
            msg = SwarmState(uav_id=0, valid=True)
            msg.timestamp.sec = stamp
            node.state(msg)
            assert node.states[0].timestamp.sec == stamp
        msg = SwarmState(uav_id=0, valid=True)
        msg.timestamp.sec = 200
        node.state(msg)
        assert node.states[0].timestamp.sec == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

