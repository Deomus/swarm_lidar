"""Единый режим группы по ТЗ §8 из состояний узлов; команд не выдаёт."""

OFFBOARD, AUTO_LAND = '14', '18'  # VehicleStatus.nav_state PX4 строкой


def system_mode(drones, formation, leader, forming_error=.6):
    """Возвращает (режим, причина).

    drones — {uav_id: dict(armed, nav_state, health)} только со свежей
    телеметрией; formation и leader — свежие JSON-статусы строя и ведущего
    или None. Порядок проверок задаёт приоритет: отказ важнее посадки,
    посадка важнее штатных режимов.
    """
    failsafe = [f'uav_{i}: {d["health"]}' for i, d in sorted(drones.items())
                if d['health'].startswith('FAILSAFE')]
    if failsafe:
        return 'FAILSAFE', 'эскалация моста — ' + ', '.join(failsafe)
    if formation is not None and formation.get('mode') in ('COLLISION_STOP', 'INVALID_GEOMETRY'):
        return 'FAILSAFE', f'строй: {formation["mode"]}'
    missing = [f'uav_{i}' for i in range(4) if i not in drones]
    if missing:
        return 'DEGRADED', 'нет свежей телеметрии: ' + ', '.join(missing)
    armed = [i for i in range(4) if drones[i]['armed']]
    if not armed:
        return 'DISARMED', 'все аппараты на земле и разоружены'
    landing = [f'uav_{i}' for i in armed if drones[i]['nav_state'] == AUTO_LAND]
    if landing:
        return 'LAND', 'посадка: ' + ', '.join(landing)
    waiting = [f'uav_{i}' for i in range(4) if i not in armed or drones[i]['nav_state'] != OFFBOARD]
    if waiting:
        return 'TAKEOFF', 'взлёт и подготовка: вне Offboard ' + ', '.join(waiting)
    if formation is None:
        return 'DEGRADED', 'нет свежего статуса строя'
    mode = formation.get('mode')
    if mode in ('DATA_HOLD', 'DEGRADED'):
        lost = formation.get('lost') or []
        return 'DEGRADED', f'строй {mode}, выбыли: ' + (', '.join(f'uav_{i}' for i in lost) or 'все')
    if mode == 'DISABLED':
        return 'FORMING', 'Offboard включён, контур строя ещё не включён'
    if mode == 'FORMATION_CHANGE':
        return 'FORMATION_CHANGE', f'перестроение: {formation.get("formation")}, scale={formation.get("scale")}'
    errors = formation.get('errors') or []
    if len(errors) != 3 or any(e is None for e in errors) or max(errors) >= forming_error:
        worst = max((e for e in errors if e is not None), default=float('nan'))
        return 'FORMING', f'ведомые занимают места, ошибка до {worst:.2f} м'
    leader_mode = (leader or {}).get('mode')
    if leader_mode == 'MISSION':
        return 'MISSION', f'маршрут, точка {leader.get("waypoint", 0) + 1}/{leader.get("total", 0)}'
    if leader_mode == 'MANUAL':
        return 'FOLLOWING', 'ведомые сопровождают ведущего в ручном режиме'
    return 'HOLD', f'ведущий: {(leader or {}).get("status", "нет статуса")}'
