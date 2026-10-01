import unittest

from robot_tests.runner import _matches_expected


class RunnerExpectationTest(unittest.TestCase):
    def test_success_requires_nav2_goal_reached(self):
        self.assertTrue(_matches_expected('success', 'goal_reached'))
        self.assertFalse(_matches_expected('success', 'timeout'))

    def test_generic_failure_does_not_hide_timeouts(self):
        self.assertTrue(_matches_expected('failure', 'invalid_goal'))
        self.assertTrue(_matches_expected('failure', 'nav2_failure'))
        self.assertFalse(_matches_expected('failure', 'timeout'))

    def test_expected_timeout_is_a_distinct_outcome(self):
        self.assertTrue(_matches_expected('timeout', 'timeout'))
        self.assertFalse(_matches_expected('timeout', 'nav2_failure'))


if __name__ == '__main__':
    unittest.main()
