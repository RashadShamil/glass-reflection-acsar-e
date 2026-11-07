"""Small semantic checks: missing evidence, planar geometry and optional pairing."""
import unittest
import numpy as np
from analyze_scans import features


class FeatureChecks(unittest.TestCase):
    def test_invalid_records_and_flat_plane(self):
        p = np.zeros(26, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4')])
        u, v = np.meshgrid(np.linspace(-.1, .1, 5), np.linspace(-.1, .1, 5))
        p['x'][:25], p['y'][:25], p['z'][:25] = u.ravel(), v.ravel(), 2
        xyz = np.column_stack([p[k] for k in 'xyz']).astype(float)
        valid = np.arange(26) < 25
        f = features(p, xyz, valid)
        self.assertTrue(np.allclose(f['curvature'][:25], 0))
        self.assertTrue(np.isnan(f['anomaly_score'][25]))
        self.assertTrue(np.isnan(f['intensity_deviation']).all())
        np.testing.assert_array_equal(f['original_point_index'], np.arange(26))

    def test_pairing_requires_timestamp_and_layout(self):
        p = np.zeros(32, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
                                ('intensity', 'f4'), ('ring', 'u2'), ('timestamp', 'f8')])
        p['ring'] = np.tile(np.arange(16), 2)
        p['x'][:16], p['x'][16:] = 2, 4
        p['timestamp'] = 100
        xyz = np.column_stack([p[k] for k in 'xyz']).astype(float)
        valid = np.ones(32, dtype=bool)
        off = features(p, xyz, valid)
        on = features(p, xyz, valid, candidate_pairs=True)
        self.assertTrue((off['candidate_partner_index'] == -1).all())
        np.testing.assert_allclose(on['candidate_pair_distance_m'], 2)
        np.testing.assert_allclose(on['anomaly_score'], off['anomaly_score'], equal_nan=True)
        p['timestamp'][16:] += 1
        rejected = features(p, xyz, valid, candidate_pairs=True)
        self.assertTrue((rejected['candidate_partner_index'] == -1).all())


if __name__ == '__main__':
    unittest.main()
