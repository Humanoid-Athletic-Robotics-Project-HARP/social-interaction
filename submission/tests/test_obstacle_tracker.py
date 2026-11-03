"""Geometry, association and missing-observation regressions for the tracker."""

import math
import unittest
import numpy as np

from harp.obstacle_tracker import STRUCTURE_ID, Tracker


def square(x, y, half=0.12):
    return np.array([[x + dx, y + dy] for dx in np.linspace(-half, half, 5)
                     for dy in np.linspace(-half, half, 5)])


def front(cx, cy, radius, origin, count=24):
    """Returns from the visible half of a circular body, seen from origin."""
    away = np.arctan2(cy-origin[1], cx-origin[0])+np.pi
    angles = away+np.linspace(-np.pi/2, np.pi/2, count)
    return np.column_stack((cx+radius*np.cos(angles), cy+radius*np.sin(angles)))


class Tracking(unittest.TestCase):
    def test_visible_side_fits_the_body_not_its_sensor_facing_box(self):
        origin = np.array([0., 0.])
        track = Tracker().update(front(3., 1., .2, origin), 0., origin)[0]
        # The returns' bounding box sits ~0.1 m toward the sensor; the fit does not.
        self.assertLess(np.hypot(track.x-3., track.y-1.), .03)
        self.assertLess(track.radius, .2+.06+.03)
        self.assertGreater(track.radius, .2)

    def test_a_new_walkers_velocity_converges_within_a_few_scans(self):
        tracker, origin = Tracker(), np.array([0., 0.])
        for step in range(4):
            t = .1*step
            track = tracker.update(front(3.+.6*t, 1., .2, origin), t, origin)[0]
        self.assertAlmostEqual(track.vx, .6, delta=.12)
        self.assertLess(abs(track.vy), .1)

    def test_a_column_never_inherits_a_passing_walkers_motion(self):
        tracker, origin = Tracker(), np.array([0., 0.])
        column = square(3., 0., .2)
        for step in range(15):
            tracker.update(column, .1*step, origin)
        column_id = tracker.tracks_at(.1*14)[0].id
        # A walker brushes past: their returns merge into the column's component.
        for step in range(15, 30):
            t = .1*step
            walker = front(3.-.6*(t-1.5)+.6, .45, .2, origin)
            tracks = {k.id: k for k in tracker.update(np.vstack((column, walker)), t, origin)}
            if column_id in tracks:
                self.assertLess(np.hypot(tracks[column_id].vx, tracks[column_id].vy), .15)

    def test_a_walker_beside_a_wall_stays_a_moving_object(self):
        from harp.obstacle_tracker import STRUCTURE_ID
        tracker, origin = Tracker(), np.array([0., -1.])
        wall = np.column_stack((np.arange(2., 6., .03), np.full(134, 1.8)))
        for step in range(45):
            t = .1*step
            walker = front(.3+.6*t, 1.5, .2, origin)   # surface ~0.1 m from the wall face
            tracks = tracker.update(np.vstack((wall, walker)), t, origin)
        moving = [k for k in tracks if k.id < STRUCTURE_ID and k.last_seen == t
                  and abs(k.x-(.3+.6*t)) < .3]
        self.assertEqual(len(moving), 1)
        self.assertAlmostEqual(moving[0].vx, .6, delta=.15)

    def test_a_sliding_visible_piece_of_a_known_wall_is_not_a_mover(self):
        # Seen whole, the wall is structure. Then people in front hide all but a
        # short piece, which slides as they walk: that piece is still the wall.
        tracker, origin = Tracker(), np.array([0., 0.])
        wall = np.column_stack((np.arange(1., 6., .03), np.full(167, 1.8)))
        for step in range(5):
            tracker.update(wall, .1*step, origin)
        for step in range(5, 25):
            start = 1.5+.1*(step-5)
            piece = wall[(wall[:, 0] >= start) & (wall[:, 0] <= start+.6)]
            tracks = tracker.update(piece, .1*step, origin)
            movers = [k for k in tracks if k.id < STRUCTURE_ID and math.hypot(k.vx, k.vy) > .15]
            self.assertEqual(movers, [])

    def test_walls_seen_while_walking_stay_static_and_bounded(self):
        tracker = Tracker()
        ids, counts = None, []
        for step in range(60):
            # A walking robot sees a sliding 12 m window of two hallway walls.
            x = .1*step
            wall = np.arange(x-6., x+6., .05)
            points = np.vstack((np.column_stack((wall, np.full(len(wall), 1.8))),
                                np.column_stack((wall, np.full(len(wall), -1.8))),
                                square(x+2., .3*np.sin(step/5.))))
            tracks = tracker.update(points, .1*step)
            walls = [t for t in tracks if abs(abs(t.y)-1.8) < .05]
            self.assertTrue(all(t.vx == t.vy == 0. for t in walls))
            seen = {t.id for t in walls if t.last_seen == .1*step}
            if ids is not None:
                # Re-observed tiles keep their IDs instead of spawning new ones.
                self.assertTrue(seen & ids)
            ids = seen
            counts.append(len(tracks))
        # Steady while walking: no ID churn accumulating tiles or ghosts.
        self.assertLessEqual(max(counts[30:])-min(counts[30:]), 6)
        self.assertEqual(len([t for t in tracks if abs(t.y) < 1.]), 1)

    def test_occluded_track_cannot_steal_a_continuously_observed_object(self):
        tracker = Tracker()
        initial = tracker.update(np.vstack((square(2.,0.),square(3.2,0.))),0.)
        visible_id = min(initial,key=lambda t: t.x).id
        hidden_id = max(initial,key=lambda t: t.x).id
        # One object remains exactly where measured; the other is occluded.
        # Growing age must not make the hidden ID preferable to the exact match.
        for step in range(1,11):
            stamp = step*.1
            tracks = {t.id:t for t in tracker.update(square(2.,0.),stamp)}
            with self.subTest(stamp=stamp):
                self.assertEqual(tracks[visible_id].last_seen,stamp)
                self.assertAlmostEqual(tracks[visible_id].x,2.)
                self.assertAlmostEqual(tracks[visible_id].vx,0.)
                self.assertEqual(tracks[hidden_id].last_seen,0.)
                self.assertAlmostEqual(tracks[hidden_id].x,3.2)
                self.assertAlmostEqual(tracks[hidden_id].vx,0.)

    def test_occluded_track_reassociates_when_it_returns_near_prediction(self):
        tracker = Tracker()
        for i in range(10):
            last = tracker.update(square(2.+.05*i,0.),.1*i)[0]
        tracker.update(np.empty((0,2)),1.3)
        returned = tracker.update(square(2.+.5*1.4,0.),1.4)
        self.assertEqual(len(returned),1)
        self.assertEqual(returned[0].id,last.id)
        self.assertEqual(returned[0].last_seen,1.4)
        self.assertAlmostEqual(returned[0].vx,.5,delta=.05)

    def test_constant_velocity_and_stable_ids(self):
        tracker = Tracker()
        ident = None
        for i in range(25):
            tracks = tracker.update(square(2 + i * 0.05, 1), i * 0.1)
            self.assertEqual(len(tracks), 1)
            ident = tracks[0].id if ident is None else ident
            self.assertEqual(tracks[0].id, ident)
        self.assertAlmostEqual(tracks[0].vx, 0.5, delta=0.02)
        self.assertAlmostEqual(tracks[0].vy, 0.0)

    def test_static_obstacle_and_large_wall_are_preserved(self):
        tracker = Tracker()
        points = np.vstack((square(2, 1), np.column_stack((np.linspace(-3, 3, 80),
                                                         np.full(80, 4)))))
        for i in range(5):
            tracks = tracker.update(points, i * 0.1)
        self.assertGreater(len(tracks), 2)
        self.assertLess(max(t.radius for t in tracks), 1)
        for point in points:
            self.assertTrue(any(np.linalg.norm(point - [t.x, t.y]) <= t.radius for t in tracks))
        self.assertTrue(all(abs(t.vx) + abs(t.vy) < 1e-8 for t in tracks))

    def test_two_objects_keep_ids_when_point_order_changes(self):
        tracker = Tracker()
        first = tracker.update(np.vstack((square(2, -1), square(3, 1))), 0)
        second = tracker.update(np.vstack((square(3.02, 1), square(2.02, -1)))[::-1], 0.1)
        self.assertEqual([(t.id, t.y) for t in first], [(t.id, t.y) for t in second])

    def test_merge_and_separation_do_not_multiply_ghost_tracks(self):
        tracker = Tracker()
        maximum = 0
        for i in range(100):
            half_gap = .5+.3*np.cos(i*.08)
            points = np.vstack((square(2,half_gap),square(2,-half_gap)))
            tracks = tracker.update(points,i*.1)
            maximum = max(maximum,len(tracks))
            for point in points:
                self.assertTrue(any(np.linalg.norm(point-[t.x,t.y]) <= t.radius for t in tracks))
        self.assertLessEqual(maximum,4)

    def test_occlusion_coasts_and_inflates_before_expiry(self):
        tracker = Tracker()
        for i in range(10):
            seen = tracker.update(square(2 + 0.05 * i, 0), 0.1 * i)[0]
        missed = tracker.update(np.empty((0, 2)), 1.2)[0]
        self.assertEqual(missed.id, seen.id)
        self.assertGreater(missed.x, seen.x)
        self.assertGreater(missed.uncertainty, seen.uncertainty)
        self.assertEqual(missed.last_seen, seen.last_seen)
        self.assertEqual(tracker.tracks_at(1.2), tracker.tracks_at(1.2))
        self.assertEqual(tracker.tracks_at(4), [])

    def test_single_return_is_not_discarded(self):
        track = Tracker().update([[1, 2]], 0)[0]
        self.assertGreater(track.radius, 0)

    def test_filtered_circle_contains_all_observed_points(self):
        tracker = Tracker()
        tracker.update(square(1, 1), 0)
        points = square(1.2, 1.1, 0.2)
        track = tracker.update(points, 0.1)[0]
        self.assertLessEqual(np.max(np.linalg.norm(points - [track.x, track.y], axis=1)),
                             track.radius)

    def test_bad_timestamps_and_nonfinite_data_fail_without_clearing(self):
        tracker = Tracker()
        expected = tracker.update(square(2, 0), 1)
        for stamp in (1, 0.5, float('nan')):
            with self.assertRaises(ValueError):
                tracker.update(square(2, 0), stamp)
        with self.assertRaises(ValueError):
            tracker.update([[float('nan'), 0]], 1.1)
        self.assertEqual(tracker.tracks_at(1), expected)

    def test_capacity_excess_raises_instead_of_dropping_obstacles(self):
        for tracker, excessive in (
                (Tracker(max_points=2), [[0, 0], [1, 1], [2, 2]]),
                (Tracker(max_cells=1), [[0, 0], [1, 1]]),
                (Tracker(max_tracks=1), [[0, 0], [3, 3]])):
            previous = tracker.update([[0, 0]], 0)
            with self.assertRaises(ValueError):
                tracker.update(excessive, 0.1)
            self.assertEqual(tracker.tracks_at(0), previous)


if __name__ == '__main__':
    unittest.main()
