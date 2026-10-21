#!/usr/bin/env python3
"""Сводка независимых полётов: разброс метрик и сравнение настроек."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import statistics

from analyze_scenario import analyze


def distribution(values):
    values = [value for value in values if value is not None]
    return dict(count=len(values), mean=statistics.mean(values) if values else None,
                sample_std=statistics.stdev(values) if len(values) > 1 else None,
                min=min(values) if values else None, max=max(values) if values else None)


def read_run(directory):
    directory = Path(directory)
    configuration = directory/'configuration.json'
    flight = directory/'flight' if (directory/'flight').is_dir() else directory
    result = json.loads((flight/'result.json').read_text())
    active = json.loads((flight/'metrics/summary.json').read_text())['active_control']
    config = json.loads(configuration.read_text())
    fingerprint = hashlib.sha256(json.dumps(
        [config['parameters'], config['world_sha256']], sort_keys=True).encode()).hexdigest()
    with (flight/'metrics/samples.csv').open(newline='') as handle:
        rows = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(handle)]
    windows = analyze(result['events'], rows)['windows']
    speeds = {str(window['requested_speed_mps']): window['metrics']['active_control']['formation_error_m']
              for window in windows if window['kind'] == 'manual_input'}
    delay = None
    for event in result['events']:
        match = re.search(r'нулевые команды подтверждены через ([\d.]+) с', event['event'])
        if match:
            delay = float(match[1])
    transitions_path = flight/'transitions.json'
    return dict(source=str(flight), scenario=result['scenario'], passed=result['passed'],
                passed_landing=result['passed_landing'], runtime_revision=config['runtime_revision'],
                configuration_sha256=fingerprint, formation=active['formation_error_m'],
                localization=active['localization_error_m'],
                min_pair_distance_m=active['min_pair_distance_m']['min'],
                active_rows=active['rows'], relative_stale_share=active['relative_stale_share'],
                leader_state_stale_share=active['leader_state_stale_share'],
                formation_missing_share=active['formation_missing_share'],
                localization_missing_share=active['localization_missing_share'],
                manual_windows=speeds, confirmed_zero_delay_s=delay,
                transitions=json.loads(transitions_path.read_text())['transitions']
                    if transitions_path.exists() else None)


def summarize_runs(runs):
    if len({run['configuration_sha256'] for run in runs}) != 1:
        raise ValueError('Внутри серии различаются параметры или мир')
    if len({run['scenario'] for run in runs}) != 1:
        raise ValueError('Разные сценарии нельзя объединять в одну серию')
    # Неудачные опыты остаются видимыми, а не исключаются из отчёта.
    result = dict(runs=runs, all_passed=all(run['passed'] for run in runs),
                formation_rms_m=distribution([run['formation']['rms'] for run in runs]),
                formation_max_m=distribution([run['formation']['max'] for run in runs]),
                min_pair_distance_m=distribution([run['min_pair_distance_m'] for run in runs]),
                localization_rms_m=distribution([run['localization']['rms'] for run in runs]),
                confirmed_zero_delay_s=distribution([run['confirmed_zero_delay_s'] for run in runs]))
    if all(run.get('transitions') is not None for run in runs):
        keys = [[(item['number'], item['formation'], item['scale']) for item in run['transitions']]
                for run in runs]
        if any(key != keys[0] for key in keys):
            raise ValueError('В повторениях различаются последовательности перестроений')
        result['transition_summary'] = []
        for index, (number, formation, scale) in enumerate(keys[0]):
            transitions = [run['transitions'][index] for run in runs]
            durations = [item.get('settling_duration_s') for item in transitions]
            result['transition_summary'].append(dict(number=number, formation=formation, scale=scale,
                settling_duration_s=distribution(durations),
                unsettled_runs=sum(value is None for value in durations)))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, nargs='+', required=True)
    parser.add_argument('--candidate', type=Path, nargs='+', required=True)
    parser.add_argument('--reformations', type=Path, nargs='*', default=[])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    baseline = summarize_runs([read_run(path) for path in args.baseline])
    candidate = summarize_runs([read_run(path) for path in args.candidate])
    if baseline['runs'][0]['scenario'] != candidate['runs'][0]['scenario']:
        parser.error('Для сравнения нужны одинаковые сценарии')
    report = dict(baseline=baseline, candidate=candidate,
                  note='Среднее и выборочное стандартное отклонение метрик отдельных полётов; '
                       'это не доверительный интервал и не доказательство надёжности.')
    for metric in ('formation_rms_m', 'formation_max_m'):
        old, new = baseline[metric]['mean'], candidate[metric]['mean']
        report[metric+'_reduction_percent'] = 100*(old-new)/old if old else None
    if args.reformations:
        report['reformations'] = summarize_runs([read_run(path) for path in args.reformations])
    with args.output.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(f'Сравнение сохранено: {args.output}')


if __name__ == '__main__':
    main()
