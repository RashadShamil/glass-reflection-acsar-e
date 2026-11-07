> **M2 reflection features (Rashad):** Start with [m2/README.md](m2/README.md) for setup, scan visualization, feature extraction and the three-day plan. The older format and gait descriptions below are historical; see the newer validation reports for corrections.

# ACSAR-E M1 — Dataset Preparation Workspace

**Project:** ACSAR-E — Gait-Aware LiDAR Reflection Confidence for Glass Surfaces  
**Module:** M1 — Dataset Preparation, Timestamp Synchronization, Sensor Calibration  
**Robot:** Unitree Go2 EDU  
**Environment:** ROS2 Foxy on Ubuntu 20.04 (WSL2)  

---

## Quick Start

```bash
# In Ubuntu-20.04 WSL:
cd /mnt/d/glass-research/acsar_e_m1

# Phase 2: Inspect PointCloud2 field structures from bag 4
python3 scripts/inspect_pointcloud_fields.py

# Phase 3: Extract 5 sample scans from bag 4 (default)
python3 scripts/extract_sample.py --n-scans 5

# Extract from a different bag or more scans:
python3 scripts/extract_sample.py \
    --bag /mnt/d/glass-research/rosbag2_2026_10_01-16_52_39 \
    --n-scans 20 \
    --out /mnt/d/glass-research/acsar_e_m1/exports
```

---

## Workspace Structure

```
acsar_e_m1/
├── scripts/
│   ├── inspect_pointcloud_fields.py   # Phase 2: CDR field inspector
│   └── extract_sample.py              # Phase 3: Sample point cloud extractor
├── reports/
│   ├── bag_inventory.csv              # Phase 1: All 6 bags compared
│   ├── pointcloud_fields.csv          # Phase 2: L1 and Hesai field details
│   ├── scan_index.csv                 # Phase 3: Index of exported scans
│   └── timestamp_report.md            # Phase 4: Timestamp analysis
├── calibration/                       # Phase 5: Calibration inputs (TBD)
├── exports/
│   ├── l1_raw/      scan_NNNN.npz    # /utlidar/cloud scans
│   ├── l1_deskewed/ scan_NNNN.npz    # /utlidar/cloud_deskewed scans
│   └── hesai/       scan_NNNN.npz    # /lidar_points (Hesai XT-16) scans
└── README.md
```

---

## Source Bag Locations

| Bag | Path | Size |
|-----|------|------|
| rosbag2_2026_10_01-16_39_53 | `/mnt/d/glass-research/rosbag2_2026_10_01-16_39_53/` | 328 MB |
| rosbag2_2026_10_01-16_41_04 | `/mnt/d/glass-research/rosbag2_2026_10_01-16_41_04/` | 232 MB |
| rosbag2_2026_10_01-16_42_00 | `/mnt/d/glass-research/rosbag2_2026_10_01-16_42_00/` | 13 GB |
| **rosbag2_2026_10_01-16_49_33** | `/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/` | 5.4 GB ← **recommended** |
| rosbag2_2026_10_01-16_51_04 | `/mnt/d/glass-research/rosbag2_2026_10_01-16_51_04/` | 1.7 GB |
| rosbag2_2026_10_01-16_52_39 | `/mnt/d/glass-research/rosbag2_2026_10_01-16_52_39/` | 8.7 GB |

> **Source bags are read-only. Never modify or delete the .db3 files.**

---

## NPZ File Format

Each `.npz` export file contains:

| Array key | Shape / type | Description |
|-----------|-------------|-------------|
| `points` | `(N, M) float32` | Point data, one column per field |
| `field_names` | `object array of strings` | Column names matching `points` |
| `header_stamp_ns` | `int64 scalar` | Sensor header timestamp (ns) |
| `bag_receipt_ns` | `int64 scalar` | ROS bag receipt timestamp (ns) |
| `frame_id` | `bytes` | TF frame the cloud is in |
| `height` | `int32` | PointCloud2 height (1 = unordered) |
| `width` | `int32` | PointCloud2 width (= N for height=1) |

### Reading an NPZ file

```python
import numpy as np

data = np.load('exports/l1_raw/scan_0000.npz', allow_pickle=True)
points      = data['points']           # (N, M) float32
field_names = list(data['field_names'])
hdr_ts      = int(data['header_stamp_ns'])
bag_ts      = int(data['bag_receipt_ns'])
frame_id    = data['frame_id'].item().decode()

print(f"Fields: {field_names}")
print(f"Points: {points.shape}")
# e.g. for L1 raw: ['x', 'y', 'z', 'intensity', 'ring', 'time']
```

---

## LiDAR Field Reference (Verified from Bag 4)

### Unitree L1 — `/utlidar/cloud` (raw)

| Field | Offset | Type | Notes |
|-------|--------|------|-------|
| x | 0 | FLOAT32 | metres |
| y | 4 | FLOAT32 | metres |
| z | 8 | FLOAT32 | metres |
| _(padding)_ | 12 | — | 4 bytes unused |
| intensity | 16 | FLOAT32 | reflectivity |
| ring | 20 | UINT16 | scan line index |
| time | 24 | FLOAT32 | per-point time offset (s) |
| _(padding)_ | 28 | — | 4 bytes to reach point_step=32 |

**Frame ID:** `utlidar_lidar` | **Point step:** 32 B | **Scan rate:** ~15 Hz | **Points/scan:** ~1,100–1,200

### Unitree L1 — `/utlidar/cloud_deskewed` (motion-compensated)

| Field | Offset | Type | Notes |
|-------|--------|------|-------|
| x | 0 | FLOAT32 | metres (in `odom` frame) |
| y | 4 | FLOAT32 | metres |
| z | 8 | FLOAT32 | metres |
| _(padding)_ | 12 | — | 4 bytes unused |
| intensity | 16 | FLOAT32 | reflectivity |

**Frame ID:** `odom` | **Point step:** 32 B | **is_dense:** False | **Points/scan:** ~10,000–10,600  
⚠️ Deskewed cloud **loses `ring` and `time` fields** — per-point timing is not preserved after FAST-LIO2 processing.

### Hesai XT-16 — `/lidar_points`

| Field | Offset | Type | Notes |
|-------|--------|------|-------|
| x | 0 | FLOAT32 | metres |
| y | 4 | FLOAT32 | metres |
| z | 8 | FLOAT32 | metres |
| intensity | 12 | FLOAT32 | reflectivity |
| ring | 16 | UINT16 | channel index (0–15) |
| timestamp | 18 | FLOAT64 | per-point sensor time |

**Frame ID:** `hesai_lidar` | **Point step:** 26 B | **Points/scan:** 64,000 | **Scan rate:** ~10 Hz

---

## Timestamp Warning

The three LiDAR streams use **different clock domains**:

| Stream | Clock | Offset from PC clock |
|--------|-------|---------------------|
| `/utlidar/cloud` header | Robot RTC / FAST-LIO2 | ~−609 s |
| `/utlidar/cloud_deskewed` header | Robot RTC / FAST-LIO2 | ~−609 s |
| `/lidar_points` header | Hesai firmware counter | ~−1,787,546,374 s |

**Use `bag_receipt_ns` as the common ordering clock for a first prototype.**  
See `reports/timestamp_report.md` for full analysis.

---

## Dependencies

No special ROS2 packages beyond the standard installation are required:

```
python3 (≥3.8, stdlib: sqlite3, struct, csv, argparse)
numpy (≥1.20)
```

No `rosbag2_py`, no `rclpy`, no `ros2bag` CLI needed to run these scripts.

---

## For M2 / M3 / M4 Teams

- **M2 (Reflection detection):** Use `exports/l1_raw/` and `exports/hesai/` with the `intensity` field.  
- **M3 (Gait-aware fusion):** Use `scan_index.csv` to match scans by `bag_receipt_ns`. Do not use header stamps for cross-sensor alignment without offset correction.  
- **M4 (Ground truth):** Use `exports/l1_deskewed/` for spatial alignment; note it loses per-point time and ring fields.

---

*ACSAR-E M1 — Dataset Preparation Module*
