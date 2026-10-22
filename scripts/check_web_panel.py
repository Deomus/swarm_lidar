#!/usr/bin/env python3
"""Проверка браузера; --flight явно включает полёты в работающей симуляции."""

import argparse
import json
import math
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://localhost:8080')
    parser.add_argument('--flight', action='store_true')
    parser.add_argument('--output', type=Path,
                        default=Path('log')/time.strftime('web_browser_%Y%m%d_%H%M%S'))
    args = parser.parse_args()
    from playwright.sync_api import sync_playwright

    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    passed = False
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True)
        page = browser.new_page(viewport=dict(width=1280, height=1100))
        errors = []
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.goto(args.url)
        token = page.locator('meta[name=panel-token]').get_attribute('content')

        def state():
            response = page.request.get(args.url+'/api/state')
            assert response.ok, 'Сервер не возвращает состояние'
            return response.json()

        def wait(predicate, timeout, reason, fail_on_error=True):
            end, previous = time.monotonic()+timeout, None
            while time.monotonic() < end:
                value = state()
                flight = value['flight']
                last = (flight['logs'] or ['Ожидание подключения'])[-1]
                if last != previous:
                    print(last, flush=True)
                    previous = last
                if predicate(value):
                    return value
                if fail_on_error and flight['phase'] == 'error':
                    raise RuntimeError('\n'.join(flight['logs'][-15:]))
                page.wait_for_timeout(500)
            raise RuntimeError(reason)

        def click(name):
            page.get_by_role('button', name=name, exact=True).click(timeout=10000)

        def leader():
            return state()['drones'][0]

        def land():
            response = page.request.post(args.url+'/api/command', data=dict(action='land'),
                                         headers={'X-Panel-Token': token})
            assert response.ok, response.text()

        try:
            page.wait_for_timeout(800)
            page.screenshot(path=str(args.output/'desktop.png'), full_page=True)
            page.set_viewport_size(dict(width=390, height=844))
            page.wait_for_timeout(200)
            assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth')
            page.screenshot(path=str(args.output/'mobile.png'), full_page=True)
            page.set_viewport_size(dict(width=1280, height=1100))
            if args.flight:
                wait(lambda s: s['connected'], 90., 'Нет четырёх аппаратов')
                for i in range(4):
                    page.locator(f'#takeoff-{i}').click(timeout=15000)
                    page.wait_for_timeout(700)
                wait(lambda s: s['flight']['phase'] == 'ready' and s.get('formation_settled', False)
                     and (s['formation'] or {}).get('mode') == 'FOLLOWING', 300., 'Строй не собрался после взлёта')
                for name in ('wedge', 'line'):
                    page.locator(f'label[for="formation-{name}"]').click()
                    page.locator('#duration').fill('25')
                    click('Применить строй')
                    wait(lambda s: s['formation'] is not None
                         and s['formation']['formation'] == name
                         and s['formation']['mode'] == 'FOLLOWING'
                         and s.get('formation_settled', False)
                         and len(s['formation']['errors']) == 3
                         and max(s['formation']['errors']) < .6, 120., 'Не установлен выбранный строй')
                    page.locator('#formation-status').filter(has_text='Строй установлен').wait_for()
                    page.screenshot(path=str(args.output/(name+'.png')), full_page=True)
                # Высота всего роя: ведущий меняет только Z, строй сохраняется.
                altitude = round(leader()['position'][2] + 1., 1)
                page.locator('#altitude').fill(str(altitude))
                click('Изменить')
                wait(lambda s: (s['execution'] or {}).get('status') == 'COMPLETE'
                     and abs(s['drones'][0]['position'][2]-altitude) < .4
                     and s.get('formation_settled', False), 120., 'Высота роя не изменена')
                # Пульт: удержание «вперёд» 3 с, затем торможение и «Стоп».
                start = leader()['position']
                box = page.locator('#move-forward').bounding_box()
                page.mouse.move(box['x']+box['width']/2, box['y']+box['height']/2)
                page.mouse.down()
                page.wait_for_timeout(3000)
                manual_status = (state()['execution'] or {}).get('status')
                page.mouse.up()
                assert manual_status == 'MANUAL', manual_status
                wait(lambda s: math.hypot(*s['drones'][0]['velocity'][:2]) < .15, 30., 'Ведущий не затормозил')
                moved = math.dist(start[:2], leader()['position'][:2])
                assert moved > .3, moved
                results.append(dict(altitude=altitude, manual_moved_m=moved))
                page.locator('#manual-stop').click()
                wait(lambda s: s['action']['state'] == 'ok' and s['execution'] is not None
                     and s['execution'].get('status') == 'HOLD', 15., 'Нет подтверждения удержания')
                click('Посадить всех')
                land()  # Повтор запроса не должен прервать посадку.
                value = wait(lambda s: s['flight']['phase'] in ('complete', 'cancelled')
                             and s['connected'] and all(not d['armed'] for d in s['drones']),
                             120., 'Посадка не подтверждена')
                results.append(value['flight'])
                page.screenshot(path=str(args.output/'finished.png'), full_page=True)
            assert not errors, errors
            passed = True
        finally:
            if args.flight:
                value = state()
                if value['flight']['phase'] in ('starting', 'running', 'ready', 'landing'):
                    land()
                    wait(lambda s: s['connected'] and all(not d['armed'] for d in s['drones']),
                         120., 'После ошибки проверки не подтверждена посадка', fail_on_error=False)
            (args.output/'browser_result.json').write_text(json.dumps(
                dict(passed=passed, flight_requested=args.flight, runs=results, javascript_errors=errors),
                ensure_ascii=False, indent=2)+'\n')
            browser.close()
    print(f'Браузерная проверка завершена: {args.output}', flush=True)


if __name__ == '__main__':
    main()
