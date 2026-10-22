#!/usr/bin/env python3
"""Проверка выбора точки; --flight отправляет цель готовому рою в SITL."""

import argparse
import json
import math
from pathlib import Path
import re
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://localhost:8080')
    parser.add_argument('--flight', action='store_true')
    parser.add_argument('--prepare', action='store_true', help='Самостоятельно подготовить операторский полёт')
    parser.add_argument('--land', action='store_true', help='Посадить группу после проверки, включая ошибку')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if (args.prepare or args.land) and not args.flight:
        parser.error('--prepare и --land требуют --flight')
    from playwright.sync_api import sync_playwright

    args.output.mkdir(parents=True, exist_ok=False)
    result = dict(passed=False, flight=args.flight, samples=[], errors=[])
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page(viewport=dict(width=1280, height=1100))
        page.on('pageerror', lambda error: result['errors'].append(str(error)))
        try:
            page.goto(args.url)
            page.locator('#connection').filter(has_text='Четыре аппарата подключены').wait_for(timeout=30000)
            if args.prepare:
                page.locator('#prepare').click(timeout=15000)
                deadline = time.monotonic()+240.
                while time.monotonic() < deadline:
                    state = page.request.get(args.url+'/api/state').json()
                    if state['flight']['phase'] == 'ready':
                        result['operator_output'] = state['flight']['output']
                        break
                    if state['flight']['phase'] == 'error':
                        raise RuntimeError('Подготовка завершилась ошибкой: '+str(state['flight']['logs'][-8:]))
                    page.wait_for_timeout(500)
                else:
                    raise RuntimeError('Группа не подготовилась за отведённое время')
            page.wait_for_timeout(800)
            canvas = page.locator('#map')
            rect = canvas.bounding_box()
            page.mouse.move(rect['x']+rect['width']/2, rect['y']+rect['height']/2)
            page.mouse.wheel(0, -120)
            page.wait_for_timeout(200)
            canvas.click(position=dict(x=rect['width']*.6, y=rect['height']*.45))
            label = page.locator('#target-label').inner_text()
            assert 'Цель:' in label, label
            result['label'] = label
            if args.flight:
                state = page.request.get(args.url+'/api/state').json()
                result['requested_altitude'] = state['drones'][0]['position'][2]
                page.locator('#go-to-target').click(timeout=15000)
                saw_mission = False
                deadline = time.monotonic()+150.
                while time.monotonic() < deadline:
                    state = page.request.get(args.url+'/api/state').json()
                    row = dict(time=state['time'], action=state['action'],
                               execution=state['execution'], drones=state['drones'])
                    result['samples'].append(row)
                    status = (state['execution'] or {}).get('status')
                    saw_mission |= status == 'MISSION'
                    if state['action']['state'] == 'error':
                        raise RuntimeError(state['action']['message'])
                    if saw_mission and status == 'COMPLETE':
                        x, y = map(float, re.findall(r'[XY]=(-?\d+\.\d+)', label))
                        position = state['drones'][0]['position']
                        result['horizontal_error'] = math.hypot(position[0]-x, position[1]-y)
                        result['altitude_error'] = abs(position[2]-result['requested_altitude'])
                        assert result['horizontal_error'] < .65
                        assert result['altitude_error'] < .65
                        break
                    page.wait_for_timeout(500)
                else:
                    raise RuntimeError('Маршрут не завершился за отведённое время')
                assert saw_mission
            page.screenshot(path=str(args.output/'map.png'), full_page=True)
            assert not result['errors'], result['errors']
            result['passed'] = True
        finally:
            if args.land:
                result['landing_confirmed'] = False
                try:
                    token = page.locator('meta[name=panel-token]').get_attribute('content')
                    response = page.request.post(args.url+'/api/command', data=dict(action='land'),
                                                 headers={'X-Panel-Token': token})
                    if not response.ok:
                        raise RuntimeError('Панель отклонила посадку')
                    deadline = time.monotonic()+100.
                    while time.monotonic() < deadline:
                        state = page.request.get(args.url+'/api/state').json()
                        if state['connected'] and all(not d.get('armed') for d in state['drones']):
                            result['landing_confirmed'] = True
                            break
                        page.wait_for_timeout(500)
                    if not result['landing_confirmed']:
                        raise RuntimeError('Посадка не подтверждена')
                except Exception as error:
                    result['passed'] = False
                    result['errors'].append(str(error))
            (args.output/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
            browser.close()
    print(json.dumps({k:v for k,v in result.items() if k != 'samples'}, ensure_ascii=False), flush=True)
    if not result['passed']:
        raise RuntimeError('Проверка не завершена; см. result.json')


if __name__ == '__main__':
    main()
