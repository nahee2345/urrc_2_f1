import unittest
from pathlib import Path
from team_11.race_context import TRACKS,configure,neutral_context,lane_relative_to_ego

class RaceContextTests(unittest.TestCase):
    def test_no_reserved_node_context_assignment(self):
        node_source=(Path(__file__).parent/'team_11'/'context_node.py').read_text()
        self.assertNotIn('self.context=',node_source)
        self.assertIn('self.race_session=configure(track,grid)',node_source)

    def test_all_tracks_11_slots(self):
        for track in TRACKS:
            for grid in range(1,12):
                with self.subTest(track=track,grid=grid):
                    session=configure(track,grid)
                    self.assertEqual(session.initial_lane,1 if grid%2 else 3)
                    self.assertEqual(lane_relative_to_ego(session.initial_lane),
                                     -1 if grid%2 else 1)
    def test_reject_missing_grid(self):
        for slot in (0,12,-1,1.5,True,None):
            with self.assertRaises(ValueError):
                configure("monza",slot)
    def test_reject_unknown_track(self):
        with self.assertRaises(ValueError):configure("unknown",1)
    def test_no_unverified_auto_guess(self):
        self.assertIsNone(neutral_context())

if __name__=="__main__":unittest.main()
