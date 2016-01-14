#!/usr/bin/python
"""
Copyright (c) 2014 NavPy Developers. All rights reserved.
Use of this source code is governed by a BSD-style license that can be found in
LICENSE.txt
"""
import navpy
import unittest
import numpy as np
assert_almost_equal = np.testing.assert_almost_equal

"""
Tests to Add
 - check constants agains WGS84
 - define a setup function to load all available lla, ecef data
"""


class TestNavClass(unittest.TestCase):

    #  def setUp(self):

    def test_lla2ned(self):
        """
        Test conversion from LLA to NED.

        Data Source: Example generated using book GNSS Applications and Methods
                     Chapter 7 library functions: wgslla2xyz.m, wgsxyz2enu
        """
        # A point near Los Angeles, CA
        lat_ref = +(34. + 0./60 + 0.00174/3600)  # North
        lon_ref = -(117. + 20./60 + 0.84965/3600)  # West
        alt_ref = 251.702  # [meters]

        # Point near by with known NED position
        lat = +(34. + 0./60 + 0.19237/3600)  # North
        lon = -(117. + 20./60 + 0.77188/3600)  # West
        alt = 234.052  # [meters]

        ned = [5.8738, 1.9959, 17.6498]

        # Do conversion and check result
        # Note: default assumption on units is deg and meters
        ned_computed = navpy.lla2ned(lat, lon, alt, lat_ref, lon_ref, alt_ref)

        for e1, e2 in zip(ned_computed, ned):
            self.assertAlmostEqual(e1, e2, places=3)        

    def test_ned2lla(self):
        """
        Test conversion from NED to LLA.

        Data Source: derived from "test_lla2ned" above.
        """
        # A point near Los Angeles, CA
        lat_ref = +( 34. + 0./60 + 0.00174/3600)  # North
        lon_ref = -(117. + 20./60 + 0.84965/3600)  # West
        alt_ref = 251.702 # [meters]  

        # Point near by with known NED position
        lat = +(34.  + 0./60 + 0.19237/3600) # North
        lon = -(117. +20./60 + 0.77188/3600) # West
        alt =  234.052 # [meters]

        ned = [5.8738, 1.9959, 17.6498]

        # Do conversion and check result
        # Note: default assumption on units is deg and meters
        lla_computed = navpy.ned2lla(ned, lat_ref, lon_ref, alt_ref)
        
        for e1, e2 in zip(lla_computed, [lat, lon, alt]):
            self.assertAlmostEqual(e1, e2, places=3)        


    def test_ecef2ned(self):
        """
        Test conversion from ECEF to NED.
        
        Data Source: Examples 2.1 and 2.4 of Aided Navigation: GPS with High
                     Rate Sensors, Jay A. Farrel 2008
        Note: N, E (+) and S, W (-)
        """
        # A point near Los Angeles, CA, given from equation 2.12 [degrees]
        lat = +( 34. +  0./60 + 0.00174/3600) # North
        lon = -(117. + 20./60 + 0.84965/3600) # West
        alt = 251.702 # [meters]

        # Define example ECEF vector and associated NED, given in Example 2.4
        ecef = np.array([0.3808, 0.7364, -0.5592])
        ned  = np.array([0, 0, 1]) # local unit gravity
        
        # Do conversion and check result
        # Note: default assumption on units is deg and meters
        ned_computed = navpy.ecef2ned(ecef, lat, lon, alt) 
        
        for e1, e2 in zip(ned_computed, ned):
            self.assertAlmostEqual(e1, e2, places=3)

    def test_ned2ecef(self):
        """
        Test conversion from NED to ECEF.
        
        Data Source: derived from "test_ecef2ned" above.
        """
        # A point near Los Angeles, CA, given from equation 2.12 [degrees]
        lat = +( 34. +  0./60 + 0.00174/3600) # North
        lon = -(117. + 20./60 + 0.84965/3600) # West
        alt = 251.702 # [meters]

        # Define example ECEF vector and associated NED, given in Example 2.4
        ecef = np.array([0.3808, 0.7364, -0.5592])
        ned  = np.array([0, 0, 1]) # local unit gravity
        
        # Do conversion and check result
        # Note: default assumption on units is deg and meters
        ecef_computed = navpy.ned2ecef(ned, lat, lon, alt) 
        
        for e1, e2 in zip(ecef_computed, ecef):
            self.assertAlmostEqual(e1, e2, places=3)

    def test_lla2ecef_Ausralia(self):
        """
        Test conversion of LLA to ECEF.
        
        Data Source: Exercise 2.4 and 2.5 of Aided Navigation: GPS with High
                      Rate Sensors, Jay A. Farrel 2008
        Note: N, E (+) and S, W (-)
        """
        # Sydney Australia
        lat = -( 33. + 53./60 + 28.15/3600) # South
        lon = +(151. + 14./60 + 57.07/3600) # East
        alt = 86.26 # [meters]
        
        # Note: The book's example only seems reliable to 2 digits passed
        #       decimal.  For example, ecef_x is recorded as xxx.571, but
        #       even using the books equation (2.7 and 2.9) you get xxx.572555
        ecef = [-4.646678571e6, 2.549341033e6, -3.536478881e6] # [meters]
        
        # Do conversion and check result
        # Note: default units are deg
        ecef_computed = navpy.lla2ecef(lat, lon, alt)
        
        for e1, e2 in zip(ecef_computed, ecef):
            self.assertAlmostEqual(e1, e2, places=2)    

    def test_ecef2lla_Ausralia(self):
        """
        Test conversion of ECEF to LLA.
        
        Data Source: Exercise 2.4 and 2.5 of Aided Navigation: GPS with High
                      Rate Sensors, Jay A. Farrel 2008
        Note: N, E (+) and S, W (-)
        """
        # Sydney Australia
        ecef = [-4.646678571e6, 2.549341033e6, -3.536478881e6] # [meters]
                
        lat = -( 33. + 53./60 + 28.15/3600) # South
        lon = +(151. + 14./60 + 57.07/3600) # East
        alt = 86.26 # [meters]
        
        # Do conversion and check result
        # Note: default units are degrees
        lla_computed = navpy.ecef2lla(ecef)
        
        # Testing for higher precision for lat, lon
        for e1, e2 in zip(lla_computed, [lat, lon]):
            self.assertAlmostEqual(e1, e2, places=8)

        # Two digits of precision for alt (i.e. cm)
        self.assertAlmostEqual(lla_computed[2], alt, places=2)


    def test_ecef2lla_SAfrica(self):
        """
        Test conversion of ECEF to LLA.
        
        Data Source: Exercise 2.4 and 2.5 of Aided Navigation: GPS with High
                      Rate Sensors, Jay A. Farrel 2008
        Note: N, E (+) and S, W (-)
        """
        # Pretoria S. Africa
        ecef = [5.057590377e6, 2.694861463e6, -2.794229000e6] # [meters]

        lat = -(26. + 8./60 + 42.20/3600) # South
        lon = +(28. + 3./60 +  0.92/3600) # East
        alt = 1660.86 # [meters]
                
        # Do conversion and check result
        # Note: default units are degrees
        lla_computed = navpy.ecef2lla(ecef)
        
        # Testing for higher precision for lat, lon
        for e1, e2 in zip(lla_computed, [lat, lon]):
            self.assertAlmostEqual(e1, e2, places=8) 
            
        # Two digist of accuracy for alt
        self.assertAlmostEqual(lla_computed[2], alt, places=2)
    
    def test_lla2ecef_SAfrica(self):
        """
        Test conversion of LLA to ECEF.
        
        Data Source: Exercise 2.4 and 2.5 of Aided Navigation: GPS with High
                      Rate Sensors, Jay A. Farrel 2008
        Note: N, E (+) and S, W (-)
        """
        # Pretoria S. Africa
        lat = -(26. + 8./60 + 42.20/3600) # South
        lon = +(28. + 3./60 +  0.92/3600) # East
        alt = 1660.86 # [meters]
        
        ecef = [5.057590377e6, 2.694861463e6, -2.794229000e6] # [meters]
        
        # Do conversion and check result
        # Note: default units are degrees
        ecef_computed = navpy.lla2ecef(lat, lon, alt)
        
        # Note: see comment in test_lla2ecef_Ausralia() as to why
        #       only 2 digits of accuracy is being tested for the example.
        for e1, e2 in zip(ecef_computed, ecef):
            self.assertAlmostEqual(e1, e2, places=2)    
    
    def test_lla2ecef(self):
        """
        Test conversion of LLA to ECEF.
        
        Data Source: Example 2.1 of Aided Navigation: GPS with High Rate 
                     Sensors, Jay A. Farrel 2008
        """
        # Near Los Angeles, CA 
        lat = +(34.  +  0./60 + 0.00174/3600) # North
        lon = -(117. + 20./60 + 0.84965/3600) # West
        alt = 251.702 # [meters]

        ecef = [-2430601.828, -4702442.703, 3546587.358] # [meters]
        
        # Do conversion and check result
        # Note: default units are degrees
        ecef_computed = navpy.lla2ecef(lat, lon, alt)
        
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
        
        lat = +(34.  +  0./60 + 0.00174/3600) # North
        lon = -(117. + 20./60 + 0.84965/3600) # West
        alt = 251.702 # [meters]

        # Do conversion and check result
        # Note: default units are degrees
        lla_computed = navpy.ecef2lla(ecef)
        
        # Testing accuracy for lat, lon greater than alt
        for e1, e2 in zip(lla_computed, [lat, lon]):
            self.assertAlmostEqual(e1, e2, places=8)
            
        self.assertAlmostEqual(lla_computed[2], alt, places=3)
    
    def test_ecef2lla_vector(self):
        """
        Test conversion of multiple ECEF to LLA
        
        Data Source: Pretoria and Sydney, Exercise 2.4 and 2.5 of
                     Aided Navigation: GPS with High Rate Sensors, Jay A. Farrel
                     2008
        """
        ecef_pretoria = [5.057590377e6, 2.694861463e6, -2.794229000e6]#in meters
        
        lat_pretoria = -(26. + 8./60 + 42.20/3600) # South
        lon_pretoria = +(28. + 3./60 +  0.92/3600) # East
        alt_pretoria = 1660.86 # [meters]
        
        ecef_sydney = [-4.646678571e6, 2.549341033e6, -3.536478881e6] #in meters
    
        lat_sydney = -( 33. + 53./60 + 28.15/3600) # South
        lon_sydney = +(151. + 14./60 + 57.07/3600) # East
        alt_sydney = 86.26 # [meters]
        
        ecef = np.vstack((ecef_pretoria,ecef_sydney))
        
        lat = [lat_pretoria,lat_sydney]
        lon = [lon_pretoria,lon_sydney]
        alt = [alt_pretoria,alt_sydney]
        
        # Do conversion and check result
        lat_comp,lon_comp,alt_comp = navpy.ecef2lla(ecef)
    
        # Testing accuracy for lat, lon
        np.testing.assert_almost_equal(lat_comp,lat,decimal=8)
        np.testing.assert_almost_equal(lon_comp,lon,decimal=8)
        np.testing.assert_almost_equal(alt_comp,alt,decimal=2)
    
    def test_lla2ecef_vector(self):
        """
        Test conversion of multiple LLA to ECEF
        
        Data Source: Pretoria and Sydney, Exercise 2.4 and 2.5 of
                     Aided Navigation: GPS with High Rate Sensors, Jay A. Farrel
                     2008
        """
        ecef_pretoria = [5.057590377e6, 2.694861463e6, -2.794229000e6]#in meters
        
        lat_pretoria = -(26. + 8./60 + 42.20/3600) # South
        lon_pretoria = +(28. + 3./60 +  0.92/3600) # East
        alt_pretoria = 1660.86 # [meters]
        
        ecef_sydney = [-4.646678571e6, 2.549341033e6, -3.536478881e6] #in meters
        
        lat_sydney = -( 33. + 53./60 + 28.15/3600) # South
        lon_sydney = +(151. + 14./60 + 57.07/3600) # East
        alt_sydney = 86.26 # [meters]

        lat = np.array([lat_pretoria,lat_sydney])
        lon = [lon_pretoria,lon_sydney]
        alt = [alt_pretoria,alt_sydney]
    
        ecef = np.vstack((ecef_pretoria,ecef_sydney))
        
        # Do conversion and check result
        ecef_computed = navpy.lla2ecef(lat,lon,alt)
    
        # Note: see comment in test_lla2ecef_Ausralia() as to why
        #       only 2 digits of accuracy is being tested for the example.
        np.testing.assert_almost_equal(ecef_computed,ecef,decimal=2)
    
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
        R_computed = navpy.omega2rates(0, 0)
        self.assertTrue((R_expected == R_computed).all())
        
        ## ... and when order is yaw_pitch_roll
        R_expected = np.array([
                     [0,0,1],
                     [0,1,0],
                     [1,0,0]])
        R_computed = navpy.omega2rates(0, 0, euler_angles_order='yaw_pitch_roll')
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
        R_computed = navpy.omega2rates(pitch_rad, roll_rad)
        np.testing.assert_almost_equal(R_expected, R_computed)
        
        ## ... and when input is in degrees.
        R_computed = navpy.omega2rates(pitch_deg, roll_deg, input_unit='deg')
        np.testing.assert_almost_equal(R_expected, R_computed)
        
        ## ... and when order is yaw_pitch_roll
        R_expected = np.array([R_expected[2,:], 
                               R_expected[1,:], 
                               R_expected[0,:]])
        R_computed = navpy.omega2rates(pitch_rad, roll_rad, euler_angles_order='yaw_pitch_roll')
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
        R_computed = navpy.omega2rates(pitch_rad, roll_rad)
        np.testing.assert_almost_equal(R_expected, R_computed)
        
        # Test singlarity and ensure NaN returned
        pitch_rad, roll_rad = np.radians(89.6), np.radians(-3)
        # Test near, but not at, singularity
        R_computed = navpy.omega2rates(pitch_rad, roll_rad)
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
        
        np.testing.assert_almost_equal(angles1_wrap_expected, navpy.wrapToPi(angles1), decimal=5)
        np.testing.assert_almost_equal(angles2_wrap_expected, navpy.wrapToPi(angles2), decimal=5)
        
        # Boundry Conditions - current implementation is kinda unintuitive for boundry
        # conditions.  These tests can be added later to impose desired behavior
        #self.assertAlmostEqual( np.pi, nav.wrap_pi( np.pi))
        #self.assertAlmostEqual(-np.pi, nav.wrap_pi(-np.pi))
        
    def test_angle2dcm(self):
        """
        Test forming transformation from navigation to body frame based on
        specified Euler angles.
        
        Data Source: Example 1 generated using book GNSS Applications and Methods
             Chapter 7 library function: eul2Cbn.m (transpose of this used).
             Example 2 found in Performance, Stability, Dynamics, and Control
             of Airplanes, Second Edition (p. 355).
        """
        
        # Define Euler angles and expected DCMs
        checks = (
            (np.deg2rad([-83, 2.3, 13]), 6,
             np.matrix([[ 0.121771, -0.991747, -0.040132],
                        [ 0.968207,  0.109785,  0.224770],
                        [-0.218509, -0.066226,  0.973585]])),
            (np.deg2rad([-10, 20, 30]), 4,
             np.matrix([[ 0.9254,  0.3188,  0.2049],
                        [-0.1632,  0.8232, -0.5438],
                        [-0.3420,  0.4698,  0.8138]]).T),
        )

        for angles, decimal, Rnav2body_expected in checks:
            yaw, pitch, roll = angles

            Rnav2body_computed = navpy.angle2dcm(yaw, pitch, roll)
            
            np.testing.assert_almost_equal(Rnav2body_expected, Rnav2body_computed, decimal=decimal)
            
            # Test units feature
            yaw_deg, pitch_deg, roll_deg = np.rad2deg([yaw, pitch, roll])
            Rnav2body_computed = navpy.angle2dcm(yaw_deg, pitch_deg, roll_deg, input_unit='deg')
            np.testing.assert_almost_equal(Rnav2body_expected, Rnav2body_computed, decimal=decimal)

        # Test with multiple inputs
        angles = np.column_stack([a for a, d, r in checks])
        Rnav2body = navpy.angle2dcm(*angles)
        for (Rnav2body_expected, decimal), Rnav2body_computed in zip(
                [(r, d) for a, d, r in checks], Rnav2body):
            np.testing.assert_almost_equal(Rnav2body_expected, Rnav2body_computed, decimal=decimal)
        
    def test_dcm2angle(self):
        """
        Test deriving Euler angles from body -> nav transformation matrix
        (transpose of DCM).
        
        This test simply creates a DCM, transposes it (to get body -> nav) and
        attempts to recover the original angles.
        """
        # Define (expected) Euler angles and associated DCM (Rnav2body)
        yaw, pitch, roll = [-83, 2.3, 13] # degrees
        
        Rnav2body = np.array([[ 0.121771, -0.991747, -0.040132],
                              [ 0.968207,  0.109785,  0.224770],
                              [-0.218509, -0.066226,  0.973585]])

        # Make sure matrix type is handled correctly
        for nptype in np.array, np.matrix:
            yaw_C, pitch_C, roll_C = navpy.dcm2angle(nptype(Rnav2body), output_unit='deg')
                               
            # Match won't be perfect because Rnav2body is rounded.
            np.testing.assert_almost_equal([yaw_C, pitch_C, roll_C], [yaw, pitch, roll], decimal=4)

        # Check output with multiple inputs
        out = navpy.dcm2angle(np.rollaxis(
            np.repeat(Rnav2body, 5).reshape(3, 3, -1), -1), output_unit='deg')
        np.testing.assert_almost_equal(
            out, np.repeat([yaw, pitch, roll], out[0].size).reshape(3, -1), decimal=4)
        
    def test_skew_array(self):
        """
        Test formation of skew symmetric matrix from 1D array input
        """
        rho = np.array([1, 2, 3])      
        rho_x_expected = np.array([[     0 , -rho[2],  rho[1]],
                                    [ rho[2],      0 , -rho[0]],
                                    [-rho[1],  rho[0],      0 ]])
                                    
        np.testing.assert_almost_equal(rho_x_expected, navpy.skew(rho))
        
    def test_skew_list(self):
        """
        Test formation of skew symmetric matrix from 1D list input
        """
        rho = [1, 2, 3]
        rho_x_expected = np.array([[     0 , -rho[2],  rho[1]],
                                    [ rho[2],      0 , -rho[0]],
                                    [-rho[1],  rho[0],      0 ]])
                                    
        np.testing.assert_almost_equal(rho_x_expected, navpy.skew(rho))

    def test_skew_matrix(self):
        """
        Test formation of skew symmetric matrix from matrix input
        """
        rho = np.matrix([1, 2, 3])
        rho_x_expected = np.array([[     0 , -rho[0,2],  rho[0,1]],
                                    [ rho[0,2],      0 , -rho[0,0]],
                                    [-rho[0,1],  rho[0,0],      0 ]])
                                    
        np.testing.assert_almost_equal(rho_x_expected, navpy.skew(rho))
        
if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestNavClass)
    unittest.TextTestRunner(verbosity=2).run(suite)    
