import unittest

from multifloor_manager.stair_completion import StairCompletionCache


class StairCompletionTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.cache = StairCompletionCache(lambda: self.now)

    def test_correlates_only_matching_success_once(self):
        self.cache.note_goal('g1', 'stair_5f_rf_up')
        self.cache.note_result('g1', 3, 0, 41)
        self.assertIsNone(self.cache.consume('stair_3f_4f_up'))
        self.assertEqual(self.cache.consume('stair_5f_rf_up'), 41)
        self.assertIsNone(self.cache.consume('stair_5f_rf_up'))
        self.cache.note_result('g1', 3, 0, 41)
        self.assertIsNone(self.cache.consume('stair_5f_rf_up'))

    def test_result_may_arrive_before_goal(self):
        self.cache.note_result('g2', 3, 0, 11)
        self.cache.note_goal('g2', 'stair_3f_4f_up')
        self.assertEqual(self.cache.consume('stair_3f_4f_up'), 11)

    def test_rejects_failed_and_old_results(self):
        self.cache.note_goal('failed', 'stair_5f_rf_up')
        self.cache.note_result('failed', 4, 5, 7)
        self.assertIsNone(self.cache.consume('stair_5f_rf_up'))
        self.cache.note_goal('old', 'stair_5f_rf_up')
        self.cache.note_result('old', 3, 0, 8)
        self.now += 31
        self.assertIsNone(self.cache.consume('stair_5f_rf_up'))


if __name__ == '__main__':
    unittest.main()
