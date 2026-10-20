"""Построение графиков из CSV без подключения к ROS."""
import argparse
import csv
import json
from pathlib import Path

from evaluation.metrics import summarize


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('csv')
    parser.add_argument('--output', required=True)
    config = parser.parse_args(args)
    with open(config.csv, newline='') as stream:
        rows = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]
    output = Path(config.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / 'summary.json').write_text(json.dumps(summarize(rows), ensure_ascii=False, indent=2) + '\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    start = min((row['timestamp'] for row in rows), default=0.)
    for index in range(1, 4):
        target = [row for row in rows if row['target_id'] == index]
        t = [row['timestamp'] - start for row in target]
        for axis, key in zip(axes[:2], ['localization_error_m', 'formation_error_m']):
            axis.plot(t, [row[key] for row in target], label=f'uav_{index}')
    first = [row for row in rows if row['target_id'] == 1]
    axes[2].plot([row['timestamp'] - start for row in first],
                 [row['min_pair_distance_m'] for row in first])
    for axis, title in zip(axes, ['Ошибка локализации, м', 'Ошибка формации, м', 'Минимальная дистанция, м']):
        axis.set_ylabel(title)
        axis.grid(True)
    axes[0].legend()
    axes[1].legend()
    axes[2].set_xlabel('Время ROS от начала записи, с')
    figure.tight_layout()
    figure.savefig(output / 'metrics.png', dpi=150)
    plt.close(figure)
