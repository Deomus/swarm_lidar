#!/usr/bin/env python3
"""Графики разброса независимых повторов по JSON compare_repeats.py."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--transitions-output', type=Path)
    args = parser.parse_args()
    data = json.loads(args.report.read_text())
    groups = [data['baseline'], data['candidate']]
    colors = ['#777777', '#2878b5']
    fig, axes = plt.subplots(1, 3, figsize=(11, 4.5), layout='constrained')
    metrics = [('formation_rms_m', 'RMS ошибки строя, м', lambda r: r['formation']['rms']),
               ('formation_max_m', 'Максимальная ошибка, м', lambda r: r['formation']['max']),
               ('min_pair_distance_m', 'Минимальная дистанция, м', lambda r: r['min_pair_distance_m'])]
    for axis, (metric, title, extract) in zip(axes, metrics):
        for index, group in enumerate(groups):
            stats = group[metric]
            axis.bar(index, stats['mean'], color=colors[index], alpha=.65, width=.55,
                     yerr=stats['sample_std'] or 0., capsize=6)
            values = [extract(run) for run in group['runs']]
            axis.scatter(index+np.linspace(-.08, .08, len(values)), values,
                         color='#202020', s=24, zorder=3)
        axis.set_title(title, fontsize=11)
        axis.set_xticks([0, 1], ['Исходный', 'Новый'])
        axis.set_ylim(bottom=0.)
        axis.grid(axis='y', alpha=.2)
    fig.suptitle('Сопровождение: независимые полётные повторы', fontsize=14)
    fig.supxlabel('Столбец — среднее; ус — выборочное стандартное отклонение; точки — отдельные полёты', fontsize=9)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)
    if args.transitions_output:
        group = data['reformations']
        transitions = group['transition_summary']
        fig, axis = plt.subplots(figsize=(10, 5), layout='constrained')
        names = {'line': 'Линия', 'column': 'Колонна', 'wedge': 'Клин'}
        labels = [f"{item['number']}. {names[item['formation']]}\nмасштаб {item['scale']:.2g}"
                  for item in transitions]
        for index, item in enumerate(transitions):
            stats = item['settling_duration_s']
            if stats['mean'] is None:
                axis.text(index, 1., 'Не установился', rotation=90)
                continue
            axis.bar(index, stats['mean'], yerr=stats['sample_std'] or 0.,
                     color=colors[1], alpha=.65, capsize=6)
            values = [run['transitions'][index].get('settling_duration_s') for run in group['runs']]
            valid = [(shift, value) for shift, value in zip(np.linspace(-.08, .08, len(values)), values)
                     if value is not None]
            axis.scatter([index+shift for shift, value in valid], [value for shift, value in valid],
                         color='#202020', s=24, zorder=3)
            if item['unsettled_runs']:
                axis.text(index, stats['mean']+1., f"Не установились: {item['unsettled_runs']}", ha='center')
        axis.set_xticks(range(len(labels)), labels)
        axis.set_ylabel('Время от начала перехода, с')
        axis.set_title('Перестроения: ошибка всех ведомых ≤ 0,3 м в течение 1 с')
        axis.set_ylim(bottom=0.)
        axis.grid(axis='y', alpha=.2)
        fig.supxlabel('Столбец — среднее; ус — выборочное стандартное отклонение; точки — отдельные полёты', fontsize=9)
        fig.savefig(args.transitions_output, dpi=160)
        plt.close(fig)


if __name__ == '__main__':
    main()
