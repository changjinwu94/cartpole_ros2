import math
import unittest
from profile_core import *
class Tests(unittest.TestCase):
    def ref(self):return dict(boot_id='boot',captured_ns=1000000000,cart_m=.0735,position_counts=-2000,counts_per_rev=3000,gear_ratio=9.7,pulley_m=.04)
    def test_endpoint_offsets(self):self.assertEqual(targets(-2000),(9000,-11000))
    def test_span(self):self.assertAlmostEqual((targets(0)[0]-targets(0)[1])*SCALE*1000,27.4914089347079)
    def test_geometry(self):self.assertAlmostEqual(cart_position(-2000,self.ref()),.0735)
    def test_targets_within_guard(self):
        for p in targets(-2000):self.assertTrue(position_guard(p,self.ref()))
    def test_excursion_guard(self):self.assertFalse(position_guard(-2000+17000,self.ref()))
    def test_boot_rejected(self):
        with self.assertRaises(ValueError):validate_reference(self.ref(),2000000000,'other')
    def test_stale_reference(self):
        with self.assertRaises(ValueError):validate_reference(self.ref(),122000000000,'boot')
    def test_valid_reference(self):self.assertEqual(validate_reference(self.ref(),2000000000,'boot'),1.)
    def test_future_reference(self):
        with self.assertRaises(ValueError):validate_reference(self.ref(),0,'boot')
    def test_geometry_rejected(self):
        r=self.ref();r['gear_ratio']=1
        with self.assertRaises(ValueError):validate_reference(r,2000000000,'boot')
    def test_down_and_up(self):
        self.assertAlmostEqual(abs(pole_angle(0,0)),math.pi)
        self.assertAlmostEqual(pole_angle(1000,0),0.)
    def test_good_frame(self):self.assertEqual(parse_frame(b'CP1,1,5000,-4,0,0,1,0,1\n')['count'],-4)
    def test_bad_encoder(self):
        with self.assertRaises(ValueError):parse_frame(b'CP1,1,5000,-4,0,0,1,1,1\n')
    def test_incomplete(self):
        with self.assertRaises(ValueError):parse_frame(b'CP1,1,5000,-4,0,0,1,0,1')
    def test_command_cycle(self):self.assertEqual(PERIOD*2,.5)
if __name__=='__main__':unittest.main()
