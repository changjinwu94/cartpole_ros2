import unittest
from phase_core import DownCrossings,PhasePolicy
class Tests(unittest.TestCase):
    def test_positive_down_crossing(self):
        p=DownCrossings(0)
        self.assertIsNone(p.update(-40,1));self.assertIsNone(p.update(-1,2))
        self.assertEqual(p.update(1,3)['direction'],1)
    def test_negative_down_crossing(self):
        p=DownCrossings(0);p.update(40,1);self.assertEqual(p.update(-1,2)['direction'],-1)
    def test_noise_does_not_arm(self):
        p=DownCrossings(0)
        for i,c in enumerate((-1,1,-2,2,0)):self.assertIsNone(p.update(c,i+1))
    def test_requires_excursion_to_rearm(self):
        p=DownCrossings(0);p.update(-40,1);self.assertIsNotNone(p.update(1,2));self.assertIsNone(p.update(-1,3));self.assertIsNone(p.update(1,4))
    def test_upright_wrap_is_not_down_crossing(self):
        p=DownCrossings(0);p.update(990,1);self.assertIsNone(p.update(-990,2))
    def test_bad_timestamp(self):
        p=DownCrossings(0);p.update(1,10)
        with self.assertRaises(ValueError):p.update(2,9)
    def test_initial_nudge_and_same_side_skip(self):
        p=PhasePolicy(0);self.assertEqual(p.choose(0,[])[0],1)
        self.assertIsNone(p.choose(300000000,[dict(direction=1,at_ns=299000000)])[0])
    def test_opposite_crossing_moves(self):
        p=PhasePolicy(0);p.choose(0,[])
        self.assertEqual(p.choose(300000000,[dict(direction=-1,at_ns=299000000)])[0],-1)
    def test_old_crossing_aborts(self):
        p=PhasePolicy(0);p.choose(0,[])
        with self.assertRaisesRegex(RuntimeError,'delivery'):p.choose(300000000,[dict(direction=-1,at_ns=200000000)])
    def test_too_fast_crossing_aborts(self):
        p=PhasePolicy(0);p.choose(0,[])
        with self.assertRaisesRegex(RuntimeError,'interval'):p.choose(100000000,[dict(direction=-1,at_ns=99000000)])
    def test_stalled_crossing_aborts(self):
        p=PhasePolicy(0);p.choose(0,[])
        with self.assertRaisesRegex(RuntimeError,'stalled'):p.choose(2000000001,[])
if __name__=='__main__':unittest.main()
