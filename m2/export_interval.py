#!/usr/bin/env python3
"""Read-only SQLite ROS2 PointCloud2 interval export; no ROS installation required."""
import argparse
import csv
import hashlib
import json
import sqlite3
from pathlib import Path

import numpy as np

TYPE = 'sensor_msgs/msg/PointCloud2'
FORMATS = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def cloud_arrays(msg):
    """Honor field offsets, counts, byte order and row padding; retain raw bytes."""
    names, formats, offsets = [], [], []
    endian = '>' if msg.is_bigendian else '<'
    descriptors = []
    if msg.point_step <= 0 or msg.row_step < msg.width * msg.point_step:
        raise ValueError('Invalid PointCloud2 point/row stride')
    for f in msg.fields:
        if f.name in names or f.datatype not in FORMATS or f.count <= 0:
            raise ValueError('Duplicate/unsupported PointField or invalid count')
        dtype = np.dtype(endian + FORMATS[f.datatype])
        if f.offset < 0 or f.offset + dtype.itemsize * f.count > msg.point_step:
            raise ValueError('PointField extends outside point_step')
        names.append(f.name)
        formats.append(dtype if f.count == 1 else (dtype, (f.count,)))
        offsets.append(f.offset)
        descriptors.append({'name': f.name, 'offset': int(f.offset),
                            'datatype': int(f.datatype), 'count': int(f.count)})
    dtype = np.dtype({'names': names, 'formats': formats, 'offsets': offsets,
                     'itemsize': int(msg.point_step)})
    if not {'x', 'y', 'z', 'intensity'}.issubset(names):
        raise ValueError('Expected XYZ and intensity fields for this M2 analyser')
    for key in ['x', 'y', 'z', 'intensity']:
        if dtype[key].shape:
            raise ValueError('Expected scalar ' + key)
    payload = np.asarray(msg.data, dtype=np.uint8).copy()
    if len(payload) != msg.height * msg.row_step:
        raise ValueError('Payload length disagrees with height * row_step')
    # Allocate byte records, copying each packed row byte-for-byte. Row padding is
    # retained separately in pointcloud_data; original logical index is row*width+col.
    records = np.empty((msg.height * msg.width, msg.point_step), dtype=np.uint8)
    for row in range(msg.height):
        start = row * msg.row_step
        packed = payload[start:start + msg.width * msg.point_step]
        records[row*msg.width:(row+1)*msg.width] = packed.reshape(msg.width, msg.point_step)
    points = records.reshape(-1).view(dtype)
    xyz = np.column_stack([points[k] for k in 'xyz'])
    valid = np.isfinite(xyz).all(1) & (xyz != 0).any(1)
    out = {'points': points, 'valid_mask': valid,
           'header_stamp_ns': np.array(int(msg.header.stamp.sec)*10**9 + int(msg.header.stamp.nanosec), dtype=np.int64),
           'frame_id': np.array(msg.header.frame_id.encode('utf-8')),
           'height': np.array(msg.height, dtype=np.int64), 'width': np.array(msg.width, dtype=np.int64),
           'point_step': np.array(msg.point_step, dtype=np.int64), 'row_step': np.array(msg.row_step, dtype=np.int64),
           'is_dense': np.array(msg.is_dense), 'is_bigendian': np.array(msg.is_bigendian),
           'pointcloud_data': payload, 'fields_json': np.array(json.dumps(descriptors))}
    out.update({'field_' + name: points[name].copy() for name in names})
    return out


def gap_report(stamps):
    delta = np.diff(np.asarray(stamps, dtype=np.int64))
    return {'interval_count': int(len(delta)),
            'median_ms': float(np.median(delta)/1e6) if len(delta) else None,
            'max_ms': float(delta.max()/1e6) if len(delta) else None,
            'nonpositive_count': int((delta <= 0).sum()),
            'over_200ms_count': int((delta > 200_000_000).sum())}


def export(bag, out, topic, start_s, duration_s):
    from rosbags.typesys import Stores, get_typestore
    dbs = sorted(bag.glob('*.db3')) if bag.is_dir() else [bag]
    if len(dbs) != 1 or dbs[0].suffix != '.db3' or not dbs[0].is_file():
        raise ValueError('This pilot supports exactly one uncompressed .db3 file. Split/compressed bags require a separate reader.')
    if not np.isfinite(start_s) or not np.isfinite(duration_s) or start_s < 0 or duration_s <= 0:
        raise ValueError('Require finite start >=0 and duration >0')
    if out.exists():
        raise ValueError('Output already exists. Choose a fresh --out; existing results will not be overwritten.')
    db = dbs[0].resolve()
    con = sqlite3.connect(db.as_uri() + '?mode=ro', uri=True)
    try:
        info = con.execute('SELECT id,type,serialization_format FROM topics WHERE name=?', (topic,)).fetchall()
        if len(info) != 1 or info[0][1:] != (TYPE, 'cdr'):
            raise ValueError('Expected exactly one CDR PointCloud2 topic: ' + topic)
        tid = info[0][0]
        anchor_topics = con.execute("SELECT id FROM topics WHERE name='/utlidar/cloud'").fetchall()
        if len(anchor_topics) != 1:
            raise ValueError('Reference topic /utlidar/cloud is missing or ambiguous')
        anchor = con.execute('SELECT MIN(timestamp) FROM messages WHERE topic_id=?', (anchor_topics[0][0],)).fetchone()[0]
        if anchor is None:
            raise ValueError('Reference topic has no messages')
        begin = int(anchor) + round(start_s * 10**9)
        end = begin + round(duration_s * 10**9)
        all_rows = con.execute('SELECT id,timestamp FROM messages WHERE topic_id=? ORDER BY timestamp,id', (tid,)).fetchall()
        selected = [(i,mid,int(stamp)) for i,(mid,stamp) in enumerate(all_rows) if begin <= stamp < end]
        if not selected:
            raise ValueError('No messages in selected half-open receipt-time interval')
        if selected[0][2] >= end or all_rows[-1][1] < end:
            raise ValueError('Requested interval extends beyond this topic\'s recorded coverage')
        subdir = {'/utlidar/cloud':'l1_raw', '/lidar_points':'hesai'}.get(topic)
        if subdir is None:
            raise ValueError('Pilot supports raw /utlidar/cloud or /lidar_points only')
        store = get_typestore(Stores.ROS2_FOXY)
        target = out/'exports'/subdir
        target.mkdir(parents=True)
        (out/'reports').mkdir()
        (out/'INCOMPLETE').write_text('Export not complete until this marker is removed.\n')
        rows, headers = [], []
        for ordinal, mid, receipt in selected:
            raw = con.execute('SELECT data FROM messages WHERE id=? AND topic_id=?', (mid,tid)).fetchone()[0]
            msg = store.deserialize_cdr(raw, TYPE)
            arrays = cloud_arrays(msg)
            stamp = int(arrays['header_stamp_ns'])
            headers.append(stamp)
            arrays.update({'bag_receipt_ns': np.array(receipt,dtype=np.int64),
                           'bag_msg_index': np.array(ordinal,dtype=np.int64),
                           'source_message_id': np.array(mid,dtype=np.int64),
                           'source_db_file': np.array(db.name), 'source_topic': np.array(topic),
                           'source_bag_name': np.array(bag.name if bag.is_dir() else db.stem),
                           'source_message_cdr': np.frombuffer(raw,dtype=np.uint8).copy(),
                           'epoch_tag': np.array(b'continuous_pilot')})
            filename = 'scan_{:06d}.npz'.format(ordinal)
            path = target/filename
            np.savez_compressed(path,**arrays)
            rows.append({'bag_msg_index':ordinal, 'epoch_tag':'continuous_pilot',
                         'bag_name':arrays['source_bag_name'].item(), 'db3_file':db.name,
                         'source_message_id':mid, 'source_cdr_sha256':hashlib.sha256(raw).hexdigest(),
                         'topic':topic, 'export_subdir':subdir, 'export_file':filename,
                         'header_stamp_ns':stamp, 'bag_receipt_ns':receipt,
                         'point_count':len(arrays['points']), 'valid_point_count':int(arrays['valid_mask'].sum()),
                         'frame_id':msg.header.frame_id, 'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
            print('Exported',filename, 'source SQLite ID',mid,flush=True)
        with (out/'reports/scan_index.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        report={'reference_topic':'/utlidar/cloud','reference_receipt_ns':int(anchor),
                'selection_clock':'bag receipt timestamps; not acquisition-time alignment',
                'interval_start_ns':begin,'interval_end_exclusive_ns':end,
                'start_s':start_s,'duration_s':duration_s,'source_db':str(db),
                'source_db_size_bytes':db.stat().st_size,'topic':topic,'exported_scans':len(rows),
                'source_message_order':'topic rows sorted by receipt timestamp then SQLite message ID; original SQLite ID retained',
                'receipt_gaps':gap_report([r['bag_receipt_ns'] for r in rows]),
                'header_gaps_in_receipt_order':gap_report(headers),
                'selection_start_to_first_receipt_ms':(selected[0][2]-begin)/1e6,
                'last_receipt_to_selection_end_ms':(end-selected[-1][2])/1e6,
                'max_point_time_semantics':'native values preserved; no synthetic times or deskew',
                'gap_threshold_note':'200 ms is a diagnostic count, not a validated acceptance limit',
                'status':'complete export; timing and physical interpretation still require review'}
        (out/'reports/interval.json').write_text(json.dumps(report,indent=2)+'\n')
        (out/'INCOMPLETE').unlink()
        print(json.dumps(report,indent=2))
        return report
    finally:
        con.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--topic',default='/utlidar/cloud')
    p.add_argument('--start-s',type=float,default=5)
    p.add_argument('--duration-s',type=float,default=5)
    a=p.parse_args()
    export(a.bag,a.out,a.topic,a.start_s,a.duration_s)

if __name__=='__main__': main()
