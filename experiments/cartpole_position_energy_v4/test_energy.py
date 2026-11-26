import math
import unittest
from energy_core import estimate,tapered_target,MG,INERTIA
class Tests(unittest.TestCase):
    def samples(self,rate=0.,offset=0.):
        return [dict(at_ns=1000000000+i*5000000,count=offset+rate*i*.005*2000/(2*math.pi)) for i in range(7)]
    def test_rest_down_zero_energy(self):self.assertAlmostEqual(estimate(self.samples(),0)['energy_fraction'],0.)
    def test_stationary_upright_target(self):self.assertAlmostEqual(estimate(self.samples(offset=1000),0)['energy_fraction'],1.)
    def test_causal_rate(self):self.assertAlmostEqual(estimate(self.samples(rate=10.),0)['pole_rate_rad_s'],10.)
    def test_nonzero_down_reference(self):self.assertAlmostEqual(estimate(self.samples(offset=12000),12000)['energy_fraction'],0.)
    def test_missing_recent_data_rejected(self):
        with self.assertRaises(RuntimeError):estimate([dict(at_ns=1,count=0)],0)
    def test_full_low_energy_move(self):self.assertEqual(tapered_target(0,10000,.5,1),(10000,1.))
    def test_partial_energy_move(self):
        target,f=tapered_target(0,10000,.825,1);self.assertEqual(target,3481);self.assertAlmostEqual(f,.348125)
    def test_reverse_partial(self):self.assertEqual(tapered_target(10000,0,.825,-1)[0],6519)
    def test_below_final_threshold_preserves_old_taper(self):
        target,f=tapered_target(0,20000,.72,1)
        self.assertAlmostEqual(f,.8);self.assertEqual(target,16000)
    def test_at_final_threshold_caps_move(self):
        self.assertEqual(tapered_target(0,20000,.85,1),(6400,.32))
    def test_small_remaining_fraction_is_not_increased(self):
        target,f=tapered_target(0,20000,.98,1)
        self.assertLess(f,.32);self.assertEqual(target,1143)
    def test_recorded_final_decision_reduced(self):
        target,f=tapered_target(-14323,5676,.8763323906474823,1)
        self.assertEqual(target,-7923);self.assertAlmostEqual(f,.32)
    def test_target_energy_coasts(self):self.assertIsNone(tapered_target(0,10000,1.,1)[0])
    def test_excess_energy_coasts(self):self.assertIsNone(tapered_target(0,10000,1.3,1)[0])
    def test_wrong_direction_no_move(self):self.assertIsNone(tapered_target(10001,10000,.2,1)[0])
    def test_never_exceeds_endpoint_or_increases_with_energy(self):
        previous=10000
        for i in range(151):
            target,_=tapered_target(0,10000,i/100,1);distance=target or 0
            self.assertTrue(0<=distance<=previous);previous=distance
    def test_continuity_at_transition_edges(self):
        for e in (.75,.85):
            left=tapered_target(0,100000,e-1e-8,1)[1]
            right=tapered_target(0,100000,e+1e-8,1)[1]
            self.assertLess(abs(right-left),1e-6)
    def test_mid_transition_blends(self):
        f=tapered_target(0,100000,.8,1)[1]
        self.assertAlmostEqual(f,((1-.8)/.35+.32)/2)
    def test_no_large_jump_across_old_threshold(self):
        left=tapered_target(0,100000,.799,1)[1]
        right=tapered_target(0,100000,.801,1)[1]
        self.assertLess(abs(right-left),.02)
    def test_invalid_energy_rejected(self):
        for e in (-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):tapered_target(0,10000,e,1)
if __name__=='__main__':unittest.main()
