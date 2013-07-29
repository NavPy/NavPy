#!/usr/bin/python
import nav
import unittest
import numpy as np
assert_almost_equal = np.testing.assert_almost_equal

"""
Tests to Add
 - check constants agains WGS84
 - define a setup function to load all available lla, ecef data
"""

class TestNavClass(unittest.TestCase):

    #def setUp(self):

    def test_lla2ecef_Ausralia(self):
        """
        Test conversion of LLA to ECEF.
        
        Data Source: Exercise 2.4 and 2.5 of Aided Navigation: GPS with High
                      Rate Sensors, Jay A. Farrel 2008
        Note: N, E (+) and S, W (-)
        """
        # Sydney Australia
        lat = -np.deg2rad( 33. + 53./60 + 28.15/3600) # South
        lon = +np.deg2rad(151. + 14./60 + 57.07/3600) # East
        alt = 86.26 # [meters]
        
        ecef = [-4.646678571e6, 2.549341033e6, -3.536478881e6] # [meters]
        
        # Do conversion and check result
        ecef_computed = nav.lla2ecef([lat, lon, alt])
        
        for e1, e2 in zip(ecef_computed, ecef):
            self.assertAlmostEqual(e1, e2, places=3)    

    def test_ecef2lla_Ausralia(self):
        """
        Test conversion of ECEF to LLA.
        
        Data Source: Exercise 2.4 and 2.5 of Aided Navigation: GPS with High
                      Rate Sensors, Jay A. Farrel 2008
        Note: N, E (+) and S, W (-)
        """
        # Sydney Australia
        ecef = [-4.646678571e6, 2.549341033e6, -3.536478881e6] # [meters]
                
        lat = -np.deg2rad( 33. + 53./60 + 28.15/3600) # South
        lon = +np.deg2rad(151. + 14./60 + 57.07/3600) # East
        alt = 86.26 # [meters]
        
        # Do conversion and check result
        lla_computed = nav.ecef2lla(ecef)
        
        for e1, e2 in zip(lla_computed, [lat, lon, alt]):
            self.assertAlmostEqual(e1, e2, places=9)      


    def test_ecef2lla_SAfrica(self):
        """
        Test conversion of ECEF to LLA.
        
        Data Source: Exercise 2.4 and 2.5 of Aided Navigation: GPS with High
                      Rate Sensors, Jay A. Farrel 2008
        Note: N, E (+) and S, W (-)
        """
        # Pretoria S. Africa
        ecef = [5.057590377e6, 2.694861463e6, -2.794229000e6] # [meters]

        lat = -np.deg2rad(26. + 8./60 + 42.20/3600) # South
        lon = -np.deg2rad(28. + 3./60 +  0.92/3600) # West
        alt = 1660.86 # [meters]
                
        # Do conversion and check result
        lla_computed = nav.ecef2lla(ecef)
        
        for e1, e2 in zip(lla_computed, [lat, lon, alt]):
            self.assertAlmostEqual(e1, e2, places=9) 
    
    def test_lla2ecef_SAfrica(self):
        """
        Test conversion of LLA to ECEF.
        
        Data Source: Exercise 2.4 and 2.5 of Aided Navigation: GPS with High
                      Rate Sensors, Jay A. Farrel 2008
        Note: N, E (+) and S, W (-)
        """
        # Pretoria S. Africa
        lat = -np.deg2rad(26. + 8./60 + 42.20/3600) # South
        lon = -np.deg2rad(28. + 3./60 +  0.92/3600) # West
        alt = 1660.86 # [meters]
        
        ecef = [5.057590377e6, 2.694861463e6, -2.794229000e6] # [meters]
        
        # Do conversion and check result
        ecef_computed = nav.lla2ecef([lat, lon, alt])
        
        for e1, e2 in zip(ecef_computed, ecef):
            self.assertAlmostEqual(e1, e2, places=3)    
    
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
            
    def test_omega2rates_trivial(self):
        """
        Test conversion of pqr to Euler angle rates.  
        Trivial case: pitch and roll are zero
        
        Data Source: Example generated using book GNSS Applications and Methods
                     Chapter 7 library function: omega2rates.m
        """
        # Test trivial case where pitch and roll are zero
        ## Under the default roll_pitch_yaw order
        R_expected = np.array([
                     [1,0,0],
                     [0,1,0],
                     [0,0,1]])
        R_computed = nav.omega2rates(0, 0)
        self.assertTrue((R_expected == R_computed).all())
        
        ## ... and when order is yaw_pitch_roll
        R_expected = np.array([
                     [0,0,1],
                     [0,1,0],
                     [1,0,0]])
        R_computed = nav.omega2rates(0, 0, euler_angles_order='yaw_pitch_roll')
        self.assertTrue((R_expected == R_computed).all())
        
    def test_omega2rates_units(self):
        """
        Test conversion of pqr to Euler angle rates.  
        Ensure both inputs as radians and degrees work.
        
        Data Source: Example generated using book GNSS Applications and Methods
                     Chapter 7 library function: omega2rates.m
        """
        pitch_deg, roll_deg = 30, -3
        pitch_rad, roll_rad = np.radians(pitch_deg), np.radians(roll_deg)
        # Test default case of radians       
        ## Under the default roll_pitch_yaw order
        R_expected = np.array([
                     [1, -0.0302162,   0.5765590],
                     [0,  0.9986295,   0.0523360],
                     [0, -0.0604324,   1.1531181]])
        R_computed = nav.omega2rates(pitch_rad, roll_rad)
        np.testing.assert_almost_equal(R_expected, R_computed)
        
        ## ... and when input is in degrees.
        R_computed = nav.omega2rates(pitch_deg, roll_deg, input_units='deg')
        np.testing.assert_almost_equal(R_expected, R_computed)
        
        ## ... and when order is yaw_pitch_roll
        R_expected = np.array([R_expected[2,:], 
                               R_expected[1,:], 
                               R_expected[0,:]])
        R_computed = nav.omega2rates(pitch_rad, roll_rad, euler_angles_order='yaw_pitch_roll')
        np.testing.assert_almost_equal(R_expected, R_computed)
        
    def test_omega2rates_singularities(self):
        """
        Test conversion of pqr to Euler angle rates.
        Ensure NaN returned and (user warned) when operating near singularities.
        """
        pitch_rad, roll_rad = np.radians(89), np.radians(-3)
        # Test near, but not at, singularity
        ## Under the default roll_pitch_yaw order
        R_expected = np.array([
                     [1, -2.9983249,   57.2114477],
                     [0,  0.9986295,    0.0523360],
                     [0, -2.9987817,   57.2201626]])
        R_computed = nav.omega2rates(pitch_rad, roll_rad)
        np.testing.assert_almost_equal(R_expected, R_computed)
        
        # Test singlarity and ensure NaN returned
        pitch_rad, roll_rad = np.radians(89.6), np.radians(-3)
        # Test near, but not at, singularity
        R_computed = nav.omega2rates(pitch_rad, roll_rad)
        self.assertTrue(R_computed is np.nan)        
                               
            
if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestNavClass)
    unittest.TextTestRunner(verbosity=2).run(suite)    
