# M2: exploratory LiDAR reflection features

Rashad's workspace for loading raw L1/Hesai observations, plotting scans, extracting features and exporting an exploratory anomaly score. Start with the data before changing the algorithm. ROS is not required.

## Install and start (WSL/Linux)

Run from the repository root:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r m2/requirements.txt
python m2/analyze_scans.py --inspect-only --sensor l1_raw --limit 1
python m2/analyze_scans.py --sensor l1_raw --limit 1
```

Open `m2/results/l1_raw__scan_0000_early/scan.png`. Left: recorded intensity. Right: exploratory anomaly score. Axes use the published sensor frame. No fusion or coordinate transformation is performed.

```bash
# First Hesai scan; include optional candidate-return diagnostics
python m2/analyze_scans.py --sensor hesai --limit 1 --candidate-pairs
# All 12 indexed raw L1/Hesai samples
python m2/analyze_scans.py --candidate-pairs
# Inspect neighborhood sensitivity in a separate output folder
python m2/analyze_scans.py --sensor l1_raw --radius-m 0.25 --out m2/results-radius025
# Focused feature checks
python -m unittest discover -s m2 -p 'test_*.py'
```

The default repository path is resolved from the script, so commands also work from a different working directory. `--repo PATH` supports another checkout with the same schema. Only `reports/scan_index.csv` entries for raw L1/Hesai are used; this avoids duplicate legacy exports and omits odom-frame deskewed references.

## Outputs and traceability

Each scan gets its own folder with:

* `features.npz`: every original input array and metadata, unchanged, plus original record indices, features, candidate partners and score. Original float32 time offsets and float64 Hesai timestamps remain in `points` and their supplied field arrays. Header/receipt stamps remain int64.
* `observations.csv`: one row for every input record, including invalid records, raw fields and features. Observation IDs combine bag, topic, source message index and original record index. These are identities within source scans, not persistent object IDs. CSV integers can lose precision when imported into spreadsheet numeric cells; use NPZ for computations.
* `scan.png`: two views of the cloud, with at most 15,000 records shown for display. Feature export uses every record. Invalid XYZ is omitted from plotting only; unavailable scores on valid points appear gray.

`run.json` stores source file SHA256 hashes, parameters, dependency versions, counts and score definition. Output directories are ignored by Git. Repeating a command into the same folder replaces that run's matching outputs; use distinct `--out` paths for experiments. Existing unrelated scan folders are not cleared: `run.json` identifies the current invocation's scans.

## Definitions

| Field | Calculation | Interpretation |
|---|---|---|
| range_m | Euclidean norm of XYZ | Range relative to published frame origin; check driver translation before interpreting optical distance |
| angular_range_jump_m | Median absolute range difference among up to 23 distinct nearby viewing directions within 1 degree | Depth discontinuity; doorway edges also produce this |
| angular_neighbor_count | Number of directions used | Support for the angular feature; array order is not assumed angular order |
| neighbor_count | Up to 16 spatial neighbors within 0.5 m | Support for local geometry |
| curvature | Smallest covariance eigenvalue / sum of eigenvalues | Local departure from a plane; requires at least five neighbors |
| plane_rms_m | Square root of smallest covariance eigenvalue | Local plane fit roughness, not true map error |
| intensity_deviation | abs(I-median(I)) / (1.4826*MAD(I)) within each scan | Unusual recorded intensity; unavailable if MAD=0; not calibrated reflectance |
| candidate_pair_* | Optional distance, absolute intensity difference, angle between proposed partners | Exploratory Hesai layout diagnostics; excluded from score |
| score_component_count | Number of available baseline components, 0–3 | Report alongside score; coverage can vary |

A flat wall has small variation perpendicular to the wall, hence a small smallest eigenvalue. A coherent ghost plane can also be flat. Spatial neighborhoods include duplicate records if present: repeated Hesai returns can affect support and covariance. This baseline does not claim independent-ray sampling.

Candidate pairing is opt-in: propose indices `32*b+c` and `32*b+16+c`, require both ring sequences 0..15 and equal finite timestamps. Equal timestamps and layout are not proof of a unique physical firing or strongest/last identities. Candidate partners remain recorded for zero records; disparities require both XYZ records valid. No returns are paired between sensors.

The score averages available components `clip(range_jump/0.5,0,1)`, `clip(curvature/0.1,0,1)` and `clip(intensity_deviation/5,0,1)`. Scales and equal weights are prototype choices, not learned or validated thresholds. Score 0.8 does not mean 80% chance of glass. Sensors/scenes with different coverage/distributions are not directly comparable. Invalid XYZ retains its record with NaN features/score. No data is deleted on the basis of this score.

## Three-day learning and implementation plan

| Day | Budget | Tasks | Deliverable |
|---|---:|---|---|
| 1 | 3–4 h | Inspect one L1 and one Hesai scan; understand XYZ/intensity/time/frame; run plots; trace selected original records | Two plots and notes explaining fields, valid counts, clocks and IDs |
| 2 | 4–5 h | Study angular range jumps and PCA geometry; inspect intensity; compare radius 0.25/0.5/1 m; investigate optional pairs | Feature exports and examples explaining high/low values and ordinary edges |
| 3 | 3–4 h | Run all indexed samples; inspect score coverage and examples; preserve parameters and hashes; prepare teammate handoff | Reproducible program, results and short limitations report |

This is a first feature extraction deliverable. It does not complete glass classification research. Ground-truth labels are absent, and cross-sensor calibration/synchronization remain preliminary. Request independent labels, driver return-mode/order evidence, continuous scans and validated clocks/transforms for later evaluation. Precision/recall/F1 cannot be reported yet. Keep heuristic predictions separate from independent labels.

## Which upstream documents to trust

Read the newer `gait_validation_report.md`, `calibration_validation.md`, `time_alignment_validation.md` and `ground_truth_requirements.md` for current caveats. Older main/handoff READMEs retain obsolete matrix formats, removed gait paths and stronger glass/synchronization claims. Use structured-array loading with `allow_pickle=False`. Corrected gait data lives in `exports/lowstate_validated.npz` and `exports/sportmode_validated.npz`; this program does not consume it. The accumulated stationary XYZ clouds discard per-record provenance and are not inputs here.


## Continuous bag pilot (PowerShell, no ROS installation)

The original ROS bag stays read-only. Use Python 3.10+ and install the optional
bag dependencies in the same environment used for the analyser:

```powershell
python -m pip install -r m2/requirements-bag.txt
$bag = "C:\Users\shami\OneDrive\Desktop\Ascar-E\new3\rosbag2_2026_10_01-16_49_33"
python m2/export_interval.py --bag "$bag" --start-s 5 --duration-s 5 --out m2/intervals/l1_5_10
```

All commands in this section are single-line PowerShell commands. Selection is
[5,10) recorder seconds after the first `/utlidar/cloud` receipt, not five
seconds after a sensor header timestamp. The reference is the same for optional
Hesai exports; this is only a common recording-time window, not synchronization.
This pilot supports one uncompressed `.db3` file. It refuses existing output
folders; use a new output path after a failed/previous attempt.

The exporter uses the Rosbags Foxy standard PointCloud2 deserializer and the
message's own field offsets/counts, endianness and row stride. It retains native
fields, invalid records, header and receipt nanoseconds, source SQLite message ID,
source database name, full original serialized CDR and original point payload.
Original point index is row*width+column. `bag_msg_index` is the zero-based topic
ordinal sorted by receipt timestamp then SQLite ID; it is not a hardware scan ID.
The original SQLite ID is the authoritative source-record locator.

Read `m2/intervals/l1_5_10/reports/interval.json` before analysing: it reports
receipt/header gaps in receipt order and the distances from requested interval
boundaries to first/last selected messages. A count of gaps over 200 ms is only a
diagnostic, not a validated rejection threshold. Nonpositive header gaps require
investigation. Similar header and receipt gaps are evidence to inspect, not proof
of data loss. Do not change timestamps or fill gaps to force continuity.

```powershell
Get-Content m2/intervals/l1_5_10/reports/interval.json
python m2/analyze_scans.py --repo m2/intervals/l1_5_10 --out m2/results-pilot --no-plots --no-csv
python m2/summarize_interval.py --interval m2/intervals/l1_5_10 --results m2/results-pilot
Invoke-Item "m2/results-pilot/timeline.png"
```

This exports authoritative `features.npz` for every selected scan without the
cost of a PNG and large per-point CSV per scan. `scan_summary.csv` and
`timeline.png` summarize score quantiles, coverage, range jumps and header/receipt
gaps. These are single-scan features plotted over time; no physical point tracking,
deskew, gait association, material labels or cross-sensor fusion is performed.
Scan-wide intensity normalization changes with scene content, so distribution
changes alone do not establish a causal motion effect.

To inspect the first three pilot clouds visually, use another output directory:

```powershell
python m2/analyze_scans.py --repo m2/intervals/l1_5_10 --limit 3 --out m2/results-pilot-preview
```

For an independent Hesai pilot in the same recorder window:

```powershell
python m2/export_interval.py --bag "$bag" --topic /lidar_points --start-s 5 --duration-s 5 --out m2/intervals/hesai_5_10
```

Interval tests use synthetic serialized PointCloud2 messages in SQLite and verify
half-open window bounds, exact byte/field preservation, row padding, big endian
payloads, source IDs, immutable source database and end-to-end analysis/summary.
They do not replace running the actual robot bag locally. The earlier baseline
software checks remain available via `python -m unittest discover -s m2 -p "test_*.py"`.

Rosbags type-store documentation: https://ternaris.gitlab.io/rosbags/api/rosbags.typesys.html
