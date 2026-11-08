#!/usr/bin/env python3
"""Traceable single-scan exploratory features. No ROS, labels or calibration required."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
VERSION = '0.1.0'


def load_scan(path):
    with np.load(path, allow_pickle=False) as data:
        raw = {key: data[key].copy() for key in data.files}
    points = raw['points']
    if points.ndim != 1 or not {'x', 'y', 'z', 'intensity'}.issubset(points.dtype.names or ()):
        raise ValueError('Expected a one-dimensional structured XYZ/intensity array')
    xyz = np.column_stack([points[a] for a in 'xyz']).astype(np.float64)
    valid = np.isfinite(xyz).all(1) & (xyz != 0).any(1)
    if raw['valid_mask'].shape != valid.shape or not np.array_equal(valid, raw['valid_mask']):
        raise ValueError('Supplied validity mask disagrees with finite/nonzero XYZ')
    if len(points) != int(raw['width']) * int(raw['height']):
        raise ValueError('Point count disagrees with cloud dimensions')
    return raw, xyz, valid


def features(points, xyz, valid, angle_deg=1.0, radius_m=0.5, candidate_pairs=False):
    n = len(points)
    names = ['range_m', 'angular_range_jump_m', 'angular_neighbor_count',
             'neighbor_count', 'curvature', 'plane_rms_m', 'intensity_deviation',
             'candidate_pair_distance_m', 'candidate_pair_intensity_delta',
             'candidate_pair_angle_deg', 'anomaly_score']
    result = {key: np.full(n, np.nan) for key in names}
    partner = np.full(n, -1, dtype=np.int64)
    ids = np.flatnonzero(valid)
    q = xyz[ids]
    ranges = np.linalg.norm(q, axis=1)
    result['range_m'][ids] = ranges
    if len(q) >= 2:
        directions = q / ranges[:, None]
        distances, neighbors = cKDTree(directions).query(
            directions, k=min(24, len(q)),
            distance_upper_bound=2 * np.sin(np.deg2rad(angle_deg) / 2))
        ok = (neighbors < len(q)) & (distances > 1e-6)
        for j in range(len(q)):
            near = neighbors[j, ok[j]]
            result['angular_neighbor_count'][ids[j]] = len(near)
            if len(near):
                result['angular_range_jump_m'][ids[j]] = np.median(np.abs(ranges[j] - ranges[near]))
        distances, neighbors = cKDTree(q).query(q, k=min(17, len(q)), distance_upper_bound=radius_m)
        for j in range(len(q)):
            near = neighbors[j, (neighbors[j] < len(q)) & (neighbors[j] != j)]
            result['neighbor_count'][ids[j]] = len(near)
            if len(near) < 5:
                continue
            local = np.vstack([q[j], q[near]])
            local -= local.mean(axis=0)
            eigen = np.maximum(np.linalg.eigvalsh(local.T @ local / len(local)), 0)
            if eigen.sum() > 1e-12:
                result['curvature'][ids[j]] = eigen[0] / eigen.sum()
                result['plane_rms_m'][ids[j]] = np.sqrt(eigen[0])
    intensity = points['intensity'][ids].astype(float)
    finite = np.isfinite(intensity)
    if finite.any():
        median = np.median(intensity[finite])
        mad = np.median(np.abs(intensity[finite] - median))
        if mad > 0:
            result['intensity_deviation'][ids[finite]] = np.abs(intensity[finite] - median) / (1.4826 * mad)
    # This optional layout hypothesis does not assign strongest/last identities.
    if candidate_pairs and n and n % 32 == 0 and {'ring', 'timestamp'}.issubset(points.dtype.names):
        blocks = np.arange(n).reshape(-1, 2, 16)
        a, b = blocks[:, 0, :].ravel(), blocks[:, 1, :].ravel()
        layout = np.tile(np.arange(16), n // 32)
        if np.array_equal(points['ring'][a], layout) and np.array_equal(points['ring'][b], layout):
            same_time = np.isfinite(points['timestamp'][a]) & (points['timestamp'][a] == points['timestamp'][b])
            a, b = a[same_time], b[same_time]
            partner[a], partner[b] = b, a
            both = valid[a] & valid[b]
            a, b = a[both], b[both]
            delta = np.linalg.norm(xyz[a] - xyz[b], axis=1)
            angle = np.rad2deg(np.arccos(np.clip(np.sum(xyz[a] * xyz[b], axis=1) /
                    (np.linalg.norm(xyz[a], axis=1) * np.linalg.norm(xyz[b], axis=1)), -1, 1)))
            for key, values in [('candidate_pair_distance_m', delta),
                                ('candidate_pair_intensity_delta', np.abs(points['intensity'][a].astype(float) - points['intensity'][b])),
                                ('candidate_pair_angle_deg', angle)]:
                result[key][a] = values
                result[key][b] = values
    # Pairing remains diagnostic and does not contribute to the baseline score.
    components = np.column_stack([np.clip(result['angular_range_jump_m'] / 0.5, 0, 1),
                                  np.clip(result['curvature'] / 0.1, 0, 1),
                                  np.clip(result['intensity_deviation'] / 5, 0, 1)])
    count = np.isfinite(components).sum(axis=1)
    usable = valid & (count > 0)
    result['anomaly_score'][usable] = np.nansum(components[usable], axis=1) / count[usable]
    result['score_component_count'] = count.astype(np.uint8)
    result['original_point_index'] = np.arange(n, dtype=np.int64)
    result['candidate_partner_index'] = partner
    return result


def plot_scan(path, points, xyz, valid, result):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator
    fig = plt.figure(figsize=(12, 6))
    ids = np.flatnonzero(valid)
    show = ids[::max(1, int(np.ceil(len(ids) / 15000)))]
    for panel, (values, title) in enumerate([(points['intensity'], 'Recorded intensity'),
                                           (result['anomaly_score'], 'Exploratory anomaly score')], 1):
        ax = fig.add_subplot(1, 2, panel, projection='3d')
        good = np.isfinite(values[show])
        ax.scatter(*xyz[show[~good]].T, color='gray', s=2)
        artist = ax.scatter(*xyz[show[good]].T, c=values[show[good]], cmap='viridis', s=2,
                            **({'vmin': 0, 'vmax': 1} if panel == 2 else {}))
        if len(ids):
            ax.set_box_aspect(np.maximum(np.ptp(xyz[ids], axis=0), 0.1))
        ax.zaxis.set_major_locator(MaxNLocator(3))
        ax.set(xlabel='X (m)', ylabel='Y (m)', zlabel='Z (m)', title=title)
        fig.colorbar(artist, ax=ax, shrink=0.5)
    fig.suptitle(path.parent.name + ' | sensor frame; gray = unavailable')
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=ROOT)
    parser.add_argument('--out', type=Path, default=ROOT / 'm2/results')
    parser.add_argument('--sensor', choices=['all', 'l1_raw', 'hesai'], default='all')
    parser.add_argument('--limit', type=int)
    parser.add_argument('--inspect-only', action='store_true')
    parser.add_argument('--no-plots', action='store_true', help='Export features without per-scan PNGs')
    parser.add_argument('--no-csv', action='store_true', help='Export authoritative NPZ only; omit large per-point CSVs')
    parser.add_argument('--candidate-pairs', action='store_true')
    parser.add_argument('--angle-deg', type=float, default=1.0)
    parser.add_argument('--radius-m', type=float, default=0.5)
    args = parser.parse_args()
    if not 0 < args.angle_deg < 180 or args.radius_m <= 0 or (args.limit is not None and args.limit <= 0):
        parser.error('Require positive radius/limit and an angle between 0 and 180')
    if (args.repo / 'INCOMPLETE').exists():
        parser.error('Interval export is incomplete; inspect exporter error before analysing')
    with (args.repo / 'reports/scan_index.csv').open(newline='') as stream:
        rows = [r for r in csv.DictReader(stream) if r['export_subdir'] in ('l1_raw', 'hesai')
                and (args.sensor == 'all' or r['export_subdir'] == args.sensor)]
    if args.limit:
        rows = rows[:args.limit]
    if not rows:
        parser.error('No matching scans in reports/scan_index.csv')
    summaries = []
    for row in rows:
        source = args.repo / 'exports' / row['export_subdir'] / row['export_file']
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        if row.get('sha256') and row['sha256'] != digest:
            raise ValueError('Source hash differs from interval manifest: ' + str(source))
        raw, xyz, valid = load_scan(source)
        for key in ['header_stamp_ns', 'bag_receipt_ns', 'bag_msg_index']:
            if int(raw[key]) != int(row[key]):
                raise ValueError('Index metadata mismatch: ' + str(source) + ' ' + key)
        if len(xyz) != int(row['point_count']):
            raise ValueError('Index point count mismatch: ' + str(source))
        summary = {'source': str(source.relative_to(args.repo)),
                   'sha256': digest,
                   'records': len(xyz), 'valid_records': int(valid.sum()),
                   'frame': raw['frame_id'].item().decode(),
                   'header_stamp_ns': int(raw['header_stamp_ns']),
                   'bag_receipt_ns': int(raw['bag_receipt_ns']),
                   'fields': {name: str(raw['points'].dtype[name]) for name in raw['points'].dtype.names}}
        if not args.inspect_only:
            result = features(raw['points'], xyz, valid, args.angle_deg, args.radius_m, args.candidate_pairs)
            dest = args.out / (row['export_subdir'] + '__' + source.stem)
            dest.mkdir(parents=True, exist_ok=True)
            output = dict(raw)
            output.update(result)
            output.update(source_bag_name=np.array(row['bag_name']),
                          source_topic=np.array(row['topic']),
                          source_sha256=np.array(summary['sha256']))
            np.savez_compressed(dest / 'features.npz', **output)
            names = list(raw['points'].dtype.names) + list(result)
            if not args.no_csv:
                with (dest / 'observations.csv').open('w', newline='') as stream:
                    writer = csv.writer(stream)
                    writer.writerow(['observation_id', 'header_stamp_ns', 'bag_receipt_ns', 'valid'] + names)
                    for i in range(len(xyz)):
                        oid = '{}:{}:{}:{}'.format(row['bag_name'], row['topic'], row['bag_msg_index'], i)
                        writer.writerow([oid, summary['header_stamp_ns'], summary['bag_receipt_ns'], bool(valid[i])]
                                        + [raw['points'][name][i].tolist() for name in raw['points'].dtype.names]
                                        + [result[name][i].item() for name in result])
            if not args.no_plots:
                plot_scan(dest / 'scan.png', raw['points'], xyz, valid, result)
            summary['candidate_pairs'] = int((result['candidate_partner_index'] >= 0).sum() // 2)
            summary['scored_records'] = int(np.isfinite(result['anomaly_score']).sum())
        summaries.append(summary)
        print(json.dumps(summary))
    if not args.inspect_only:
        import scipy
        import matplotlib
        manifest = {'version': VERSION, 'parameters': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
                    'dependencies': {'numpy': np.__version__, 'scipy': scipy.__version__, 'matplotlib': matplotlib.__version__},
                    'score_definition': 'Mean available clipped(range_jump/0.5, curvature/0.1, intensity_deviation/5); candidate pairs excluded',
                    'interpretation': 'Untrained anomaly score; not glass probability or ground truth', 'scans': summaries}
        (args.out / 'run.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
