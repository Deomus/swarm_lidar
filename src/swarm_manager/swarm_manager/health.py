"""Статусы определяются по возрасту измерения, а не прихода пакета."""
def data_status(now, stamp, valid, short_timeout=.3, long_timeout=1.):
    if stamp is None or not valid:
        return 'HOLD'
    age = now - stamp
    if age < 0 or age > long_timeout:
        return 'HOLD'
    if age > short_timeout:
        return 'DEGRADED'
    return 'FOLLOWING'

