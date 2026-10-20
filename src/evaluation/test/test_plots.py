import csv
import json

from evaluation.plots import main


def test_csv_to_summary_and_plot(tmp_path):
    source = tmp_path / 'samples.csv'
    row = dict(timestamp=1., target_id=1, localization_error_m=.2,
               formation_error_m=.3, min_pair_distance_m=2., relative_stale=0,
               leader_state_stale=0, gt_missing=0, formation_missing=0,
               localization_missing=0)
    with source.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(row))
        writer.writeheader()
        writer.writerow(row)
    destination = tmp_path / 'plots'
    main([str(source), '--output', str(destination)])
    assert (destination / 'metrics.png').is_file()
    summary = json.loads((destination / 'summary.json').read_text())
    assert summary['localization_error_m']['rms'] == .2
    assert summary['min_pair_distance_m']['min'] == 2.
