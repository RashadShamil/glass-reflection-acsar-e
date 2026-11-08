"""Synthetic CDR/SQLite integration checks; original robot bag is not available here."""
import contextlib
import csv
import io
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import unittest
import numpy as np
from export_interval import cloud_arrays, export
try:
    from rosbags.typesys import Stores,get_typestore
except ImportError:
    get_typestore=None


@unittest.skipIf(get_typestore is None, 'Install m2/requirements-bag.txt for interval tests')
class IntervalChecks(unittest.TestCase):
    def setUp(self):
        self.store=get_typestore(Stores.ROS2_FOXY)
        self.types=self.store.types

    def message(self,big=False):
        endian='>' if big else '<'
        dtype=np.dtype({'names':['x','y','z','intensity','ring','timestamp','vector'],
                        'formats':[endian+'f4']*4+[endian+'u2',endian+'f8',(endian+'f4',(2,))],
                        'offsets':[0,4,8,12,16,18,26], 'itemsize':36})
        # Known padding bytes and two rows with extra row padding.
        data=np.full(2*(2*36+4),173,dtype='u1')
        points=np.ndarray((2,2),dtype=dtype,buffer=data,strides=(76,36))
        points['x']=[[1,2],[0,4]];points['y']=0;points['z']=0
        points['intensity']=[[10,20],[0,40]];points['ring']=[[0,1],[2,3]]
        points['timestamp']=3307199.123456789;points['vector']=[1.25,2.5]
        fields=[self.types['sensor_msgs/msg/PointField'](name,int(dtype.fields[name][1]),
                {'x':7,'y':7,'z':7,'intensity':7,'ring':4,'timestamp':8,'vector':7}[name],2 if name=='vector' else 1) for name in dtype.names]
        header=self.types['std_msgs/msg/Header'](self.types['builtin_interfaces/msg/Time'](100,123456789),'utlidar_lidar')
        return self.types['sensor_msgs/msg/PointCloud2'](header,2,2,fields,big,36,76,data,False)

    def test_endian_padding_and_field_counts(self):
        for big in [False,True]:
            original=self.message(big)
            cdr=self.store.serialize_cdr(original,'sensor_msgs/msg/PointCloud2')
            msg=self.store.deserialize_cdr(cdr,'sensor_msgs/msg/PointCloud2')
            d=cloud_arrays(msg)
            np.testing.assert_array_equal(d['pointcloud_data'],original.data)
            np.testing.assert_array_equal(d['points']['x'],[1,2,0,4])
            np.testing.assert_array_equal(d['valid_mask'],[True,True,False,True])
            np.testing.assert_array_equal(d['points']['vector'],np.tile([1.25,2.5],(4,1)))
            expected=np.concatenate([original.data[:72],original.data[76:148]])
            self.assertEqual(d['points'].tobytes(),expected.tobytes())
            self.assertEqual(int(d['header_stamp_ns']),100123456789)

    def test_interval_export_and_analyse(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag=Path(tmp)/'bag';bag.mkdir();db=bag/'bag_0.db3'
            con=sqlite3.connect(db)
            con.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT); CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
            con.execute("INSERT INTO topics VALUES(7,'/utlidar/cloud','sensor_msgs/msg/PointCloud2','cdr')")
            for i,relative in enumerate([0,5_000_000_000,5_100_000_000,9_999_999_999,10_000_000_000,11_000_000_000]):
                msg=self.message();msg.header.stamp.sec=100+i
                raw=bytes(self.store.serialize_cdr(msg,'sensor_msgs/msg/PointCloud2'))
                con.execute('INSERT INTO messages VALUES(?,?,?,?)',(100+i,7,1_790_000_000_000_000_000+relative,raw))
            con.commit();con.close()
            before=db.read_bytes();out=Path(tmp)/'interval'
            with contextlib.redirect_stdout(io.StringIO()):report=export(bag,out,'/utlidar/cloud',5,5)
            self.assertEqual(report['exported_scans'],3)
            self.assertEqual(db.read_bytes(),before)
            self.assertFalse((out/'INCOMPLETE').exists())
            with self.assertRaises(ValueError):export(bag,out,'/utlidar/cloud',5,5)
            with (out/'reports/scan_index.csv').open() as stream:rows=list(csv.DictReader(stream))
            self.assertEqual([int(r['source_message_id']) for r in rows],[101,102,103])
            with np.load(out/'exports/l1_raw/scan_000001.npz') as d:
                recovered=self.store.deserialize_cdr(d['source_message_cdr'].tobytes(),'sensor_msgs/msg/PointCloud2')
                np.testing.assert_array_equal(recovered.data,d['pointcloud_data'])
            # Scalar standard scan fields for analyser compatibility; unknown vector
            # fields are preserved in NPZ and CSV omission is used in this pilot.
            results=Path(tmp)/'results';script=Path(__file__).with_name('analyze_scans.py')
            run=subprocess.run([sys.executable,str(script),'--repo',str(out),'--out',str(results),'--no-plots','--no-csv'],capture_output=True,text=True)
            self.assertEqual(run.returncode,0,run.stderr)
            with np.load(results/'l1_raw__scan_000001/features.npz') as d:
                self.assertEqual(int(d['source_message_id']),101)
                self.assertTrue(np.isnan(d['anomaly_score'][2]))
            run=subprocess.run([sys.executable,str(script.with_name('summarize_interval.py')),'--results',str(results),'--interval',str(out)],capture_output=True,text=True)
            self.assertEqual(run.returncode,0,run.stderr)
            self.assertTrue((results/'timeline.png').is_file())
            self.assertEqual(len(json.loads((results/'run.json').read_text())['scans']),3)

if __name__=='__main__':unittest.main()
