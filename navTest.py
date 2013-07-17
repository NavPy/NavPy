#!/usr/bin/python
import nav
import unittest
import numpy as np

"""
Tests to Add
 - check constants agains WGS84
 
"""

class TestNavClass(unittest.TestCase):

    #def setUp(self):
    
    def test_lla2ecef(self):
        """
        Test conversion of LLA to ECEF.
        
        Data Source: Example 2.1 of Aided Navigation: GPS with High Rate 
                     Sensors, Jay A. Farrel 2008
        """
        # Near Los Angeles, CA 
        lat = +np.deg2rad(34.  +  0./60 + 0.00174/3600) # North
        lon = -np.deg2rad(117. + 20./60 + 0.84965/3600) # West
        alt = 251.702 # [meters]

        ecef = [-2430601.828, -4702442.703, 3546587.358] # [meters]
        
        # Do conversion and check result
        ecef_computed = nav.lla2ecef([lat, lon, alt])
        
        for e1, e2 in zip(ecef_computed, ecef):
            self.assertAlmostEqual(e1, e2, places=3)
            
    def test_ecef2lla(self):
        """
        Test conversion of ECEF to LLA.
        
        Data Source: Example 2.2 of Aided Navigation: GPS with High Rate 
                     Sensors, Jay A. Farrel 2008
        """
        # Near Los Angeles, CA 
        ecef = [-2430601.828, -4702442.703, 3546587.358] # [meters]
        
        lat = +np.deg2rad(34.  +  0./60 + 0.00174/3600) # North
        lon = -np.deg2rad(117. + 20./60 + 0.84965/3600) # West
        alt = 251.702 # [meters]

        # Do conversion and check result
        lla_computed = nav.ecef2lla(ecef)
        
        for e1, e2 in zip(lla_computed, [lat, lon, alt]):
            self.assertAlmostEqual(e1, e2, places=9)   
if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestNavClass)
    unittest.TextTestRunner(verbosity=2).run(suite)    
