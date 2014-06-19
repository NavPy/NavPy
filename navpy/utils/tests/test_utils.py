#!/usr/bin/python
"""
Copyright (c) 2014 NavPy Developers. All rights reserved.
Use of this source code is governed by a BSD-style license that can be found in
LICENSE.txt
"""
from navpy import utils
import unittest
import numpy as np

class TestUtilsClass(unittest.TestCase):
    
    def test_inputcheckNx1_list(self):
        """
        Test Nx1 input checking function for list input
        """
        rho = [1, 2, 3]
        rho_expect, N_expect = np.array([1, 2, 3]), 3
        np.testing.assert_almost_equal(rho_expect, utils.input_check_Nx1(rho)[0])
        np.testing.assert_almost_equal(N_expect, utils.input_check_Nx1(rho)[1])
    
    def test_inputcheckNx1_array(self):
        """
        Test input checking function for a valid 2D array input
        """
        rho = np.array([[1], [2], [3]])
        rho_expect, N_expect = np.array([1, 2, 3]), 3
        np.testing.assert_almost_equal(rho_expect, utils.input_check_Nx1(rho)[0])
        np.testing.assert_almost_equal(N_expect, utils.input_check_Nx1(rho)[1])
        
    def test_inputcheckNx1_invalid_array(self):
        """
        Test input checking function for invalid 2D array input
        """
        rho = np.array([[1,2], [3,4], [5,6]])
        self.assertRaises(ValueError,utils.input_check_Nx1,rho)
    
    def test_inputcheckNx1_invalid_list(self):
        """
        Test input checking function for invalid 2D array input
        """
        rho = [[1,2], [3,4], [5,6]]
        self.assertRaises(ValueError,utils.input_check_Nx1,rho)
        
if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestUtilsClass)
    unittest.TextTestRunner(verbosity=2).run(suite)    
