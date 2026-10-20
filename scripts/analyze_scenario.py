#!/usr/bin/env python3
"""Разделение измеренных метрик по временным окнам событий сценария."""
import argparse
import csv
import json
from pathlib import Path
import re

from evaluation.metrics import summarize


def analyze(events, rows):
    events = sorted(events, key=lambda event: event['time'])
    windows = []
    for index, event in enumerate(events):
        text = event['event']
        match = re.fullmatch(r'Проверка ручного входа: ([\d.]+) м/с', text)
        if match:
            end = next((item['time'] for item in events[index+1:]
                        if item['event'].startswith(('Проверка ручного входа:', 'Потеря LiDAR-поз',
                                                      'Сценарий завершён'))), None)
            label = dict(kind='manual_input', requested_speed_mps=float(match[1]))
        elif text.startswith('Перестроение:'):
            end = next((item['time'] for item in events[index+1:]
                        if item['event'].startswith(('Формация установилась:', 'Перестроение:',
                                                      'Сценарий завершён'))), None)
            label = dict(kind='formation_change', request=text)
        else:
            continue
        if end is not None:
            samples = [row for row in rows if event['time'] <= row['timestamp'] < end]
            windows.append(dict(**label, start=event['time'], end=end,
                                duration_s=end-event['time'], metrics=summarize(samples)))
    return dict(windows=windows, overall_metrics=summarize(rows),
                note='Окна ручного входа включают разгон, повороты и торможение; '
                     'это заданные скорости команд, не измеренные постоянные скорости аппарата.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', help='Каталог с result.json и metrics/samples.csv')
    parser.add_argument('--output', required=True, help='Новый файл JSON')
    args = parser.parse_args()
    root = Path(args.scenario)
    result = json.loads((root/'result.json').read_text())
    with (root/'metrics/samples.csv').open(newline='') as stream:
        rows = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]
    report = analyze(result['events'], rows)
    report.update(scenario=result['scenario'], passed=result['passed'], source=str(root))
    with Path(args.output).open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(f'Анализ сохранён: {args.output}')


if __name__ == '__main__':
    main()
