"""
WGS 84 four defining parameters and several commonly used derived parameters.
All parameters are stored with the exact number of significant digits provided
by the WGS 84 rublished report.  Available parameters:

Parameters
----------

a : semi-major axis [m]
f : flattenning
omega_E : angular velocity of the Earth [rad/s]
GM : earth's gravitational constant [m^3/s^2]
     (note: for GPS applications, use GM_GPS)
        
_b : semi-minor axis [m]
_ecc : first eccentricity
_ecc_sqrd : first eccentricity squared

Copyright (c) 2014 NavPy Developers. All rights reserved.
Use of this source code is governed by a BSD-style license that can be found in
LICENSE.txt

References
----------
.. [1] NIMA Technical Report TR8350.2, "Department of Defense World Geodetic
       System 1984, Its Definition and Relationships With Local Geodetic Systems"

       http://earth-info.nga.mil/GandG/publications/tr8350.2/wgs84fin.pdf
       Accessed on Nov. 19, 2013 

Examples
--------
Note, examples don't print full precision.

>>> import wgs84
>>> print(wgs84.a)
6378137.0
>>> print(wgs84.f)
0.00335281066475
>>> print(wgs84.omega_E)
7.292115e-05
>>> print(wgs84.GM)
3.986004418e+14
>>> print(wgs84.GM_GPS)
3.986005e+14
>>> 
>>> print(wgs84._b)
6356752.3142
>>> print(wgs84._ecc)
0.0818191908426
>>> print(wgs84._ecc_sqrd)
0.00669437999014
"""

# Table 3.1: WGS 84  Four Defining Parameters
a = 6378137.0 # Semi-major Axis [m]
f = 1./298.257223563 # Flattening
omega_E = 7292115.0e-11 # Angular velocity of the Earth [rad/s]
omega_E_GPS = 7292115.1467e-11 # Angular velocity of the Earth [rad/s]
                              # According to ICD-GPS-200

GM = 3986004.418e8 # Earth's Gravitational Constant [m^3/s^2]
                   # (mass of earth's atmosphere included)

GM_GPS = 3986005.0e8 # The WGS 84 GM value recommended for GPS receiver usage 
                     # by the GPS interface control document (ICD-GPS-200) 
                     # differs from the current refined WGS 84 GM value.
                     #
                     # Details for this difference can be read in the WGS84 
                     # reference: 3.2.3.2 "Special Considerations for GPS"

# Table 3.3: WGS 84 Ellipsoid Derived Geometric Constants
_b = 6356752.3142 # Semi-minor axis [m]
_ecc = 8.1819190842622e-2 # First eccentricity
_ecc_sqrd = 6.69437999014e-3 # First eccentricity squared
