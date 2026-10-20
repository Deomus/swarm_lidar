from swarm_manager.health import data_status

def test_stale_transitions():
    assert data_status(1., .9, True) == 'FOLLOWING'
    assert data_status(1., .5, True) == 'DEGRADED'
    assert data_status(2., .5, True) == 'HOLD'

def test_invalid_missing_and_future():
    assert data_status(1., .9, False) == 'HOLD'
    assert data_status(1., None, True) == 'HOLD'
    assert data_status(1., 2., True) == 'HOLD'

