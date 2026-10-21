#!/usr/bin/env python3
"""Проверка браузера; --flight явно включает полёты в работающей симуляции."""

import argparse
import json
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
            scenario = {'Перестроения': 's1', 'Маршрут к точке': 's3', 'Сопровождение': 's2'}.get(name)
            if scenario:
                page.locator(f'[data-scenario="{scenario}"]').click(timeout=10000)
            else:
                page.get_by_role('button', name=name, exact=True).click(timeout=10000)

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
                click('Подготовить и взлететь')
                wait(lambda s: s['flight']['phase'] == 'ready', 240., 'Нет готовности после взлёта')
                for name in ('wedge', 'line'):
                    page.locator('#formation').select_option(name)
                    page.locator('#duration').fill('25')
                    click('Применить строй')
                    wait(lambda s: s['formation'] is not None
                         and s['formation']['formation'] == name
                         and s['formation']['mode'] == 'FOLLOWING'
                         and s.get('formation_settled', False)
                         and len(s['formation']['errors']) == 3
                         and max(s['formation']['errors']) < .6, 120., 'Не установлен выбранный строй')
                    page.wait_for_function("document.getElementById('formation-status').textContent.includes('Строй установлен')")
                    page.screenshot(path=str(args.output/(name+'.png')), full_page=True)
                click('Удерживать')
                wait(lambda s: s['action']['state'] == 'ok' and s['execution'] is not None
                     and s['execution'].get('status') == 'HOLD', 15., 'Нет подтверждения удержания')
                click('Посадить всех')
                land()  # Повтор запроса не должен прервать посадку.
                value = wait(lambda s: s['flight']['phase'] in ('complete', 'cancelled')
                             and s['connected'] and all(not d['armed'] for d in s['drones']),
                             120., 'Посадка не подтверждена')
                results.append(value['flight'])
                print('Проверка прерывания сценария во время подготовки', flush=True)
                click('Перестроения')
                wait(lambda s: s['flight']['kind'] == 's1'
                     and any(d.get('armed') for d in s['drones']),
                     150., 'Нет вооружённого аппарата для проверки прерывания')
                click('Посадить всех')
                value = wait(lambda s: s['flight']['phase'] == 'cancelled'
                             and s['connected'] and all(not d['armed'] for d in s['drones']),
                             120., 'Прерванный сценарий не подтвердил посадку')
                results.append(value['flight'])
                # После S1 ведущий около исходной точки; S3 проверяется до перемещения S2.
                for kind, label in [('s1', 'Перестроения'), ('s3', 'Маршрут к точке'), ('s2', 'Сопровождение')]:
                    print('Браузер запускает: '+label, flush=True)
                    click(label)
                    value = wait(lambda s: s['flight']['kind'] == kind and s['flight']['phase'] == 'complete'
                                 and s['connected'] and all(not d['armed'] for d in s['drones']),
                                 700., 'Сценарий не завершился')
                    results.append(value['flight'])
                    print('Пройдено: '+kind, flush=True)
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
