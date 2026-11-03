"""Opt-in RL policy is bounded; absent policy is safe fallback."""
import sys
from pathlib import Path
import tempfile
import json
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.rl_tuner import ACTIONS,action,state


class V20(unittest.TestCase):
    def test_missing_policy_returns_none(self):
        self.assertIsNone(action('',0.,0.,3.))
        self.assertIsNone(action('/not/existent.json',0.,0.,3.))

    def test_policy_action_within_fixed_bounds(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'q.json'
            p.write_text(json.dumps({'schema':1,'q':{
                state(.1,.1,3.):[0.]*(len(ACTIONS)-1)+[99.]}}))
            result=action(str(p),.1,.1,3.)
            self.assertEqual(result,ACTIONS[-1])
            self.assertLessEqual(result[0],2.6)
            self.assertLessEqual(result[1],1.05)


if __name__=='__main__':unittest.main()
