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
                               
    def test_wrap_pi(self):
        """
        Test wrapping angles to [-PI, PI]

        Data Source: Example generated using book GNSS Applications and Methods
             Chapter 7 library function: unwrapyaw.m
        """
        
        angles1 = np.radians([0, 3, 175,  179,  180.1, 189,  190, 250,  380])
        angles2 = np.radians([0, 3, 175, -179, -180.1, 189, -190, 250, -380])
        
        angles1_wrap_expected = [0.0, 0.05236, 3.05433,  3.12414, -3.13985, -2.98451, -2.96706, -1.91986,  0.34907]
        angles2_wrap_expected = [0.0, 0.05236, 3.05433, -3.12414,  3.13985, -2.98451,  2.96706, -1.91986, -0.34907]
        
        # Current problem: Demoz converts +/- 180 degrees to +pi
        
        np.testing.assert_almost_equal(angles1_wrap_expected, nav.wrap_pi(angles1), decimal=5)
        np.testing.assert_almost_equal(angles2_wrap_expected, nav.wrap_pi(angles2), decimal=5)
        
        # Boundry Conditions - current implementation is kinda unintuitive for boundry
        # conditions.  These tests can be added later to impose desired behavior
        #self.assertAlmostEqual( np.pi, nav.wrap_pi( np.pi))
        #self.assertAlmostEqual(-np.pi, nav.wrap_pi(-np.pi))
        
    def test_angle2dcm(self):
        """
        Test forming transformation from navigation to body frame based on
        specified Euler angles.
        
        Data Source: Example generated using book GNSS Applications and Methods
             Chapter 7 library function: eul2Cbn.m (transpose of this used)
        """
        
        # Define Euler angles
        yaw, pitch, roll = np.deg2rad([-83, 2.3, 13])

        Rnav2body_expected = np.matrix([[ 0.121771, -0.991747, -0.040132],
                                        [ 0.968207,  0.109785,  0.224770],
                                        [-0.218509, -0.066226,  0.973585]])

        Rnav2body_computed = nav.angle2dcm(yaw, pitch, roll)
        
        np.testing.assert_almost_equal(Rnav2body_expected, Rnav2body_computed, decimal=6)
        
        # Test units feature
        yaw_deg, pitch_deg, roll_deg = np.rad2deg([yaw, pitch, roll])
        Rnav2body_computed = nav.angle2dcm(yaw_deg, pitch_deg, roll_deg, input_units='deg')
        np.testing.assert_almost_equal(Rnav2body_expected, Rnav2body_computed, decimal=6)
    
    def test_Rbody2nav_to_angle(self):
        """
        Test deriving Euler angles from body -> nav transformation matrix
        (transpose of DCM).
        
        This test simply creates a DCM, transposes it (to get body -> nav) and
        attempts to recover the original angles.
        """
        # Define (expected) Euler angles and associated DCM (Rnav2body)
        yaw, pitch, roll = [-83, 2.3, 13] # degrees

        Rnav2body = np.matrix([[ 0.121771, -0.991747, -0.040132],
                               [ 0.968207,  0.109785,  0.224770],
                               [-0.218509, -0.066226,  0.973585]])

        Rbody2nav = Rnav2body.T
        
        yaw_C, pitch_C, roll_C = nav.Rbody2nav_to_angle(Rbody2nav, output_units='deg')
        
        # Match won't be perfect because Rnav2body is rounded.
        np.testing.assert_almost_equal([yaw_C, pitch_C, roll_C], [yaw, pitch, roll], decimal=4)
        
    def test_sk(self):
        """
        Test formation of skew symmetric matrix.
        """
        rho = np.array([1, 2, 3])      
        rho_x_expected = np.matrix([[     0 , -rho[2],  rho[1]],
                                    [ rho[2],      0 , -rho[0]],
                                    [-rho[1],  rho[0],      0 ]])
                                    
        np.testing.assert_almost_equal(rho_x_expected, nav.sk(rho))
        
    def test_euler2tilt_errors(self):
        """
        Test formation of transformation from Euler Angle error states to tilt
        angle error states.
        
        This test simply checks if the formed matrix matches the expected
        version.  The expected version was manually created in IPython based
        on the reference equation 2.80 in Jay Farrell.
        """
        # Define Euler angles ('321')
        yaw, pitch, roll = [0.7, 0.1, 0.0] # radians, (note: roll isn't used.)

        Omega_T_expected = np.matrix([[ 0.76102116, -0.64421769,  0.],
                                      [ 0.64099928,  0.76484219,  0.],
                                      [-0.09983342,  0.        ,  1.]])
        
        Omega_T = nav.euler2tilt_errors(yaw, pitch, roll)
        
        # Match won't be perfect because Rnav2body is rounded.
        np.testing.assert_almost_equal(Omega_T_expected, Omega_T, decimal=7)

if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestNavClass)
    unittest.TextTestRunner(verbosity=2).run(suite)    
