import unittest

from robot_autonomy.manager import (
    _goal_needs_progress_tracking,
    _grid_cells_within_radius,
    _pose_made_progress,
    _connected_free_cells,
)


class FrontierBlacklistTest(unittest.TestCase):
    def test_blacklists_a_map_space_neighborhood_only(self):
        cells = _grid_cells_within_radius(
            center_x=5, center_y=5, width=12, height=12,
            resolution=0.05, radius=0.11)

        self.assertIn(5 * 12 + 5, cells)
        self.assertIn(5 * 12 + 7, cells)
        self.assertNotIn(5 * 12 + 8, cells)
        self.assertNotIn(0, cells)

    def test_neighborhood_is_clipped_to_the_map(self):
        cells = _grid_cells_within_radius(
            center_x=0, center_y=0, width=4, height=3,
            resolution=0.05, radius=0.11)

        self.assertEqual(cells, {0, 1, 2, 4, 5, 8})

    def test_rotation_alone_is_not_progress_away_from_goal(self):
        reference = (0.0, 0.0, 0.0)
        current = (0.0, 0.0, 1.0)

        self.assertFalse(_pose_made_progress(reference, current, 0.20, 0.15, False))
        self.assertTrue(_pose_made_progress(reference, current, 0.20, 0.15, True))

    def test_translation_counts_as_progress(self):
        self.assertTrue(_pose_made_progress(
            (0.0, 0.0, 0.0), (0.21, 0.0, 0.0), 0.20, 0.15, False))

    def test_stationary_active_goal_is_monitored(self):
        self.assertTrue(_goal_needs_progress_tracking(True, 'EXPLORING', False))
        self.assertTrue(_goal_needs_progress_tracking(True, 'COVERING', False))

    def test_inactive_goal_or_progress_resets_stuck_timer(self):
        self.assertFalse(_goal_needs_progress_tracking(False, 'EXPLORING', False))
        self.assertFalse(_goal_needs_progress_tracking(True, 'RECOVERY', False))
        self.assertFalse(_goal_needs_progress_tracking(True, 'EXPLORING', True))

    def test_frontier_reachability_excludes_disconnected_free_islands(self):
        grid = [0, 0, 100, 0, 0,
                0, 0, 100, 0, 0,
                100, 100, 100, 0, 0]

        self.assertEqual(_connected_free_cells(grid, 5, 3, [0]), {0, 1, 5, 6})
        self.assertEqual(_connected_free_cells(grid, 5, 3, [3]), {3, 4, 8, 9, 13, 14})


if __name__ == '__main__':
    unittest.main()
