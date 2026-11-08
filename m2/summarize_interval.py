#!/usr/bin/env python3
"""Plot per-scan feature and timing summaries, without tracking surfaces or fusing sensors."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def quantile(values,q):
    values=values[np.isfinite(values)]
    return float(np.quantile(values,q)) if len(values) else float('nan')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--results',type=Path,required=True)
    p.add_argument('--interval',type=Path,required=True)
    a=p.parse_args()
    manifest=json.loads((a.results/'run.json').read_text())
    info=json.loads((a.interval/'reports/interval.json').read_text())
    anchor=int(info['reference_receipt_ns'])
    rows=[]
    for scan in manifest['scans']:
        source=Path(scan['source'])
        file=a.results/(source.parent.name+'__'+source.stem)/'features.npz'
        with np.load(file,allow_pickle=False) as d:
            if str(d['source_sha256'].item()) != scan['sha256']:
                raise ValueError('Results changed since run.json: '+str(file))
            valid=d['valid_mask'];score=d['anomaly_score'];coverage=d['score_component_count']
            row={'source':scan['source'],'source_message_id':int(d['source_message_id']),
                 'bag_msg_index':int(d['bag_msg_index']),
                 'bag_receipt_ns':int(d['bag_receipt_ns']),
                 'header_stamp_ns':int(d['header_stamp_ns']),
                 'elapsed_receipt_s':(int(d['bag_receipt_ns'])-anchor)/1e9,
                 'records':len(valid),'valid_records':int(valid.sum()),
                 'scored_records':int(np.isfinite(score).sum()),
                 'score_median':quantile(score[valid],.5),
                 'score_p90':quantile(score[valid],.9),
                 'intensity_median':quantile(d['points']['intensity'][valid],.5),
                 'range_jump_median_m':quantile(d['angular_range_jump_m'][valid],.5)}
            row.update({'components_'+str(k):int((valid & (coverage==k)).sum()) for k in range(4)})
            rows.append(row)
    if not rows: raise ValueError('No scans in analyser run manifest')
    rows.sort(key=lambda r:(r['bag_receipt_ns'],r['source_message_id']))
    # This summary is intentionally single-topic: clocks are not cross-sensor aligned.
    if len({Path(r['source']).parent.name for r in rows}) != 1:
        raise ValueError('Use a single sensor per interval summary')
    with (a.results/'scan_summary.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    t=np.array([r['elapsed_receipt_s'] for r in rows]);fig,axs=plt.subplots(4,1,figsize=(10,11),sharex=True)
    axs[0].plot(t,[r['score_median'] for r in rows],label='Median score')
    axs[0].plot(t,[r['score_p90'] for r in rows],label='90th percentile');axs[0].set(ylabel='Exploratory score',ylim=(0,1));axs[0].legend()
    for k in range(4):
        fraction=[r['components_'+str(k)]/r['valid_records'] if r['valid_records'] else np.nan for r in rows]
        axs[1].plot(t,fraction,label=str(k)+' components')
    axs[1].set(ylabel='Fraction of valid records',ylim=(0,1.05));axs[1].legend(ncol=4,fontsize=8)
    axs[2].plot(t,[r['range_jump_median_m'] for r in rows],color='teal');axs[2].set(ylabel='Median range jump (m)')
    for key,label in [('bag_receipt_ns','Receipt gaps'),('header_stamp_ns','Header gaps in receipt order')]:
        gaps=np.diff(np.array([r[key] for r in rows],dtype=np.int64))/1e6
        axs[3].plot(t[1:],gaps,marker='.',label=label)
    axs[3].set(xlabel='Recorder elapsed seconds from first L1 receipt (not acquisition time)',ylabel='Inter-scan gap (ms)');axs[3].legend(fontsize=8)
    fig.suptitle('Single-sensor interval diagnostics | no material labels or temporal correspondence')
    fig.tight_layout();fig.savefig(a.results/'timeline.png',dpi=160);plt.close(fig)
    print('Wrote scan_summary.csv and timeline.png for',len(rows),'scans')
    print('Selection and timing audit:',json.dumps(info['receipt_gaps']),json.dumps(info['header_gaps_in_receipt_order']))

if __name__=='__main__':main()
