
"""
 This module contains all the commonly used constants in Navigation
 To use this module, import it using "from CONSTANT import *"
 """

from math import pi; #Definition of pi from math library.

# WGS-84
R0 = 6378137.0;		 #Earth's Radius at the equator, semi-major.
Rp = 6356752.314245;	 #Earth's Radius at the pole, semi-minor.
f = 1/298.257223563;	 #Earth's Flattening
ecc = 0.0818191908425;	 #Earth's eccentricity
omega_E = 7.292115e-5;   #15.041 deg/hr, includes revolution about sun.
g0 = 9.780373;		 #Gravity
gc1 = 0.0052891;	 #Local Gravity Computation
gc2 = 0.0000059;	 #Local Gravity Computation

# Unit Conversions
d2r = pi/180;			 #Degree to Radian
r2d = 1/d2r;			 #Radian to Degree
f2m=0.3048; 			 #Feet to Meter
kts2ms = 0.514444444;	 #Knots to m/s

# COLUMN ID
fg_tcol = 0;	fg_latcol = 1;	fg_loncol = 2;	fg_altcol = 3;
fg_TAScol = 4;	fg_GScol = 5;
fg_phicol = 6;	fg_thecol = 7;	fg_psicol = 8;
fg_tracol = 9;	fg_fpacol = 10;
fg_pcol = 11;	fg_qcol = 12;	fg_rcol = 13;
fg_ucol = 14;	fg_vcol = 15;	fg_wcol = 16;
fg_VNcol = 17;	fg_VEcol = 18;	fg_VDcol = 19;

airdat_tcol = 0;	airdat_latcol = 1;	airdat_loncol = 2;	airdat_altcol = 3;
airdat_TAScol = 4;	airdat_VNcol = 5;	airdat_VEcol = 6;	airdat_VDcol = 7;
airdat_tracol = 8; 	airdat_fpacol = 9;
airdat_ucol = 10;	airdat_vcol = 11;	airdat_wcol = 12;
