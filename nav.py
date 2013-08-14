# A test file for several generic navigation functions

import numpy as numpy
import numpy as np
from math import sin, cos, tan, asin, acos, atan2, fabs, sqrt
from nav_CONSTANT import *

######################## ATTITUDE TOOLBOX ##############################
# Functino att2body
def att2body(eul_radians):
    """
    This function returns a transformation matrix to convert
    attitude rates (roll rate, pitch rate, and yaw rate) to 
    body rates (p, q, and r).
    Input:
      eul: Euler Angle in radian, 1x3, in the following order
	       [Roll Pitch Yaw]
    Output:
    	  R: Transformation matrix (returned as array), 3x3
    	     such that [p, q, r].T = R * [roll_rate, pitch_rate, yaw_rate].T
    
    Reference: Flight Dynamics Principles, 2nd Ed by Michael V. Cook (Eqn 2.21)
    	     
	Programmer:    Hamid Mokhtarzadeh
	Created:    	 Sept 06, 2012   
    """
    phi = eul_radians[0]; the = eul_radians[1]; psi = eul_radians[2];
        
    R = numpy.array([[1.0,       0.0,          -sin(the)],
                     [0.0,  cos(phi),  sin(phi)*cos(the)],
                     [0.0, -sin(phi),  cos(phi)*cos(the)]])
    return R

def angle2dcm(yaw, pitch, roll, input_units='rad', rotation_sequence='321'):
    """
    Returns a transformation matrix (aka direction cosine matrix or DCM) which 
    transforms from navigation to body frame.  Other names commonly used, 
    besides DCM, are `Cbody2nav` or `Rbody2nav`.  The rotation sequence 
    specifies the order of rotations when going from navigation-frame to 
    body-frame.  The default is '321' (i.e Yaw -> Pitch -> Roll).

    Parameters
    ----------
    yaw   : yaw angle, units of input_units.
    pitch : pitch angle, units of input_units.
    roll  : roll angle , units of input_units.
    input_units: units for input angles {'rad', 'deg'}, optional.
    rotationSequence: assumed rotation sequence {'321', others can be 
                                                implemented in the future}.

    Returns
    -------
    Rnav2body: 3x3 transformation matrix (numpy matrix data type).  This can be
               used to convert from navigation-frame (e.g NED) to body frame.
        
    Notes
    -----
    Since Rnav2body is a proper transformation matrix, the inverse
    transformation is simply the transpose.  Hence, to go from body->nav,
    simply use: Rbody2nav = Rnav2body.T

    Examples:
    ---------
    >>> import numpy as np
    >>> from nav import angle2dcm
    >>> g_ned = np.matrix([[0, 0, 9.8]]).T # gravity vector in NED frame
    >>> yaw, pitch, roll = np.deg2rad([90, 15, 0]) # vehicle orientation
    >>> g_body = Rnav2body * g_ned
    >>> g_body
    matrix([[-2.53642664],
            [ 0.        ],
            [ 9.4660731 ]])
            
    >>> g_ned_check = Rnav2body.T * g_body 
    >>> np.linalg.norm(g_ned_check - g_ned) < 1e-10 # should match g_ned
    True

    Reference
    ---------
    [1] Equation 2.4, Aided Navigation: GPS with High Rate Sensors, Jay A. Farrel 2008
    [2] eul2Cbn.m function (note, this function gives body->nav) at:
    http://www.gnssapplications.org/downloads/chapter7/Chapter7_GNSS_INS_Functions.tar.gz
    """
    # Apply necessary unit transformations.
    if input_units == 'rad':
        pass
    elif input_units == 'deg':
        yaw, pitch, roll = np.radians([yaw, pitch, roll])

    # Build transformation matrix Rnav2body.
    s_r, c_r = sin(roll) , cos(roll)
    s_p, c_p = sin(pitch), cos(pitch)
    s_y, c_y = sin(yaw)  , cos(yaw)

    if rotation_sequence == '321':
        # This is equivalent to Rnav2body = R(roll) * R(pitch) * R(yaw)
        # where R() is the single axis rotation matrix.  We implement
        # the expanded form for improved efficiency.
        Rnav2body = np.matrix([
                [c_y*c_p               ,  s_y*c_p              , -s_p    ],
                [-s_y*c_r + c_y*s_p*s_r,  c_y*c_r + s_y*s_p*s_r,  c_p*s_r],
                [ s_y*s_r + c_y*s_p*c_r, -c_y*s_r + s_y*s_p*c_r,  c_p*c_r]])

    else: 
        # No other rotation sequence is currently implemented
        print('WARNING (angle2dcm): requested rotation_sequence is unavailable.')
        print('                     NaN returned.')
        Rnav2body = np.nan
    
    return Rnav2body

def Rbody2nav_to_angle(R, output_units='rad', rotation_sequence='321'):
    """
    Returns a Euler angles derived from transformation matrix R (body->nav).
    This transformation matrix is the transpose of the DCM.
    The rotation sequence specifies the order of rotations which were assumed
    when forming the DCM (R.T). The default is '321' (i.e Yaw -> Pitch -> Roll).

    Parameters
    ----------
    R : 3x3 numpy array or matrix
        Transformation matrix from body -> nav (transpose of DCM)
    output_units : {'rad', 'deg'}, optional
                   Units for output Euler angles.
    rotationSequence : {'321', others can be implemented in the future}
                       Assumed rotation sequence for the supplied DCM (R.T).

    Returns
    -------
    yaw   : yaw angle, units of output_units.
    pitch : pitch angle, units of output_units.
    roll  : roll angle , units of output_units.
    
    
    Reference
    ---------
    [1] Equation 2.45-47, Aided Navigation: GPS with High Rate Sensors, Jay A. Farrel 2008
    [2] Cbn2eul.m function at:
    http://www.gnssapplications.org/downloads/chapter7/Chapter7_GNSS_INS_Functions.tar.gz        
    """
    yaw = np.arctan2(R[1,0], R[0,0])
    #pitch = -np.arctan(R[2,0] / np.sqrt(1.-R[2,0]**2)) # Farrel eqn 2.45
    pitch =  -np.arcsin(R[2,0]) #  this is simpler
    roll  = np.arctan2(R[2,1], R[2,2] )
    
    # Apply necessary unit transformations.
    if output_units == 'rad':
        pass
    elif output_units == 'deg':
        yaw, pitch, roll = np.degrees([yaw, pitch, roll])
        
    return yaw, pitch, roll
    
	
#################### EARTH PROPERTY (WGS84) ############################
# Function earthrad(lat)
def earthrad(lat):
	"""
	This function calculates the Earth Radius in both the East-West
	direction and the North South direction given the latitude.
	Using WGS-84.
	Input:
		lat: Latitude in radian
	Output: In tuples:
		Rew: Radius in East-West direction
		Rns: Radius in North-South direction
	Programmer:    Adhika Lie
	Created:    	 May 03, 2011
	Last Modified: May 03, 2011
	"""
		
	#Alternative formula:
	#Rew = R*(1 + f*sin(lat)*sin(lat));
	#Rns = R*(1 + f*(3*sin(lat)*sin(lat) - 2));
	
	Rew = R0/(1-(ecc*sin(lat))**2)**0.5;
	Rns = R0*(1-ecc**2)/(1-(ecc*sin(lat))**2)**1.5;
	return Rew, Rns

################## INERTIAL NAVIGATION FUNCTION ########################
def earthrate(lat):
	"""
	This function calculates the Earth Rotation Rate in NED coordinate
	The angular velocity (omega) includes the revolution about sun.
	Using WGS-84.
	Input:
		lat: Latitude in radian
	Output:
		e: Earth Angular Velocity in NED Coordinate, numpy array 3x1.
	Programmer:    Adhika Lie
	Created:    	 May 04, 2011
	Last Modified: May 04, 2011
	"""
		
	e = numpy.array([[0.0], [0.0], [0.0]]);
	
	e[0,0] = omega_E*cos(lat);
	e[2,0] = -omega_E*sin(lat);
	
	return e;
	
def llarate(V,lla):
	"""
	This function calculates the rate of change of latitude, longitude,
	and altitude.
	Using WGS-84.
	Input:
		V: Groundspeed in NED Coordinate, can be 1x3 or 3x1 numpy array.
		lla: [lat,lon,alt] numpy array, 1x3 or 3x1.
	Output:
		lla_dot: [Lat Rate, Lon Rate, Alt Rate]', 3x1 numpy array.
	Programmer:    Adhika Lie
	Created:    	 May 04, 2011
	Last Modified: May 04, 2011
	"""
	lat = lla[0]; h = lla[2];
	
	R = earthrad(lat);
	
	lla_dot = numpy.zeros((3,1));
	
	lla_dot[0,0] = V[0]/(R[1] + h);
	lla_dot[1,0] = V[1]/((R[0] + h)*cos(lat));
	lla_dot[2,0] = -V[2];
	
	return lla_dot;
	
def navrate(V,lla):
	"""
	This function calculates the angular velocity of the NED frame, 
	also known as the navigation rate.
	Using WGS-84.
	Input:
		V: Groundspeed in NED Coordinate, can be 1x3 or 3x1 numpy array.
		lla: [lat,lon,alt] numpy array, 1x3 or 3x1.
	Output:
		rho: angular velocity of the NED frame in NED coordinate, 
			 3x1 numpy array.
	Programmer:    Adhika Lie
	Created:    	 May 04, 2011
	Last Modified: May 04, 2011
	"""
	lat = lla[0]; h = lla[2];
	
	latlonrate = llarate(V,lla);
	
	rho = numpy.zeros((3,1));
	
	rho[0,0] = latlonrate[1,0]*cos(lat);
	rho[1,0] = -latlonrate[0,0];
	rho[2,0] = -latlonrate[1,0]*sin(lat);
	
	return rho;
	
def glocal(lla):
	"""
	This function calculates the local gravity vector. 
	Note that  gravity = gravitation + centripetal
	Using model from Demoz Gebre's toolbox
	Input:
		lla: [lat,lon,alt] numpy array, 1x3 or 3x1.
	Output:
		g: Local gravity in 3x1 numpy array.
	Programmer:    Adhika Lie
	Created:    	 May 10, 2011
	Last Modified: May 10, 2011
	"""
	lat = lla[0];
	h = lla[2];
	
	mg = g0*(1+gc1*(sin(lat))**2 - gc2*(sin(2*lat))**2);
	mg = mg/((1+h/R0)**2);
	g = numpy.zeros([3,1]);
	g[2,0] = mg;
	
	return g;

def omega2rates(pitch, roll, input_units='rad', euler_angles_order='roll_pitch_yaw'):
    """
    This function is used to create the transformation matrix to go from:
	    [p, q, r] --> [roll_rate, pitch_rate, yaw_rate]
    where pqr are xyz body rotation-rate measurements expressed in body frame.
    Yaw, pitch, and roll are the Euler angles.  We assume the Euler angles are
    3-2-1 (i.e Yaw -> Pitch -> Roll) transformations that go from navigation-
    frame to body-frame.

    Parameters
    ----------
    pitch : pitch angle, units of input_units.
    roll  : roll angle , units of input_units.
    input_units: units for input angles {'rad', 'deg'}, optional.
    euler_angles_order: assumed order of Euler Angles attitude state vector
                        {'roll_pitch_yaw', 'yaw_pitch_roll'} (see ``Notes``).

    Returns
    -------
    R: transformation matrix, from xyz body-rate to Euler angle-rates
       numpy array 3x3 (Note: the return variable is an ARRAY, not a matrix)
        
    Notes
    -----
    Since the returned transformation matrix is used to transform one vector
    to another, the assumed attitude variables order matters.  
    The ``euler_angles_order`` parameter can be used to specify the assumed order.

    The difference is demonstrated by example:
        # By default euler_angles_order='roll_pitch_yaw'
        R = omega2rates(pitch, roll) 
        [ roll_rate]         [omega_x]
        [pitch_rate] = dot(R,[omega_y])
        [  yaw_rate]         [omega_z]

        # Now assume our attitude state is [yaw, pitch, roll].T
        R = omega2rates(pitch, roll, euler_angles_order='yaw_pitch_roll') 
        [ yaw_rate]          [omega_x]
        [pitch_rate] = dot(R,[omega_y])
        [ roll_rate]         [omega_z]	

    Reference
    ---------
    [1] Equation 2.74, Aided Navigation: GPS with High Rate Sensors, Jay A. Farrel 2008
    [2] omega2rates.m function at:
    http://www.gnssapplications.org/downloads/chapter7/Chapter7_GNSS_INS_Functions.tar.gz
    """
    # Apply necessary unit transformations.
    if input_units == 'rad':
        pitch_rad, roll_rad = pitch, roll
    elif input_units == 'deg':
        pitch_rad, roll_rad = np.radians([pitch, roll])

    # Build transformation matrix.
    s_r, c_r = sin( roll_rad), cos( roll_rad)
    s_p, c_p = sin(pitch_rad), cos(pitch_rad)
    
    # Check for singularities (i.e. pitch near 90 degrees)
    singular_tol = 1e-2; # flags anything between [90 +/- .5 deg]
    if abs(c_p) < singular_tol:
        print('WARNING (omega2rates): Operating near pitch = 90 deg singularity.  NaN returned. ')
        return np.nan

    if euler_angles_order == 'roll_pitch_yaw':
        R = np.array(
           [[  1, s_r*s_p/c_p,  c_r*s_p/c_p],
            [  0, c_r        , -s_r        ],
            [  0, s_r/c_p    ,  c_r/c_p    ]], dtype=float)
    elif euler_angles_order == 'yaw_pitch_roll':
        R = np.array(
           [[  0, s_r/c_p    ,  c_r/c_p    ],
            [  0, c_r        , -s_r        ],
            [  1, s_r*s_p/c_p,  c_r*s_p/c_p]], dtype=float)

    return R

def get_pqr_rel(lla,VG_NED,e,om):
	"""
	This function is used to get the body rate with respect to the
	Local Level Navigation NED frame.
	Input:
	 	 lla: Location [lat(rad) lon(rad) alt(m)]
	VG_NED: NED Ground Speed in m/s
		   e: Euler Angle in [phi the psi] format
		  om: Absolute body rate (w.r.t inertial frame) pqr in rad/s
	Output:
	pqr_rel: Body Rate with respect to the local NED frame, 3x1.
	Programmer:    Adhika Lie
	Created:    	 May 13, 2011
	Last Modified: May 13, 2011
	"""
	om_E = earthrate(lla[0]);
	om_E = om_E.reshape(3,1);
	
	om_N = navrate(VG_NED,lla);
	om_N = om_N.reshape(3,1);
	C_N2B = eul2dcm(e);
	
	Om = numpy.dot(C_N2B,om_N+om_E);
	
	pqr_rel = om.reshape(3,1)-Om;
	#pqr_rel = pqr_rel.reshape(3,1);
	
	return pqr_rel;


###################### COORDINATE TRANSFORMATION #######################
# Function lla2ecef
def lla2ecef(lla):
	"""
	This function calculates the ECEF Coordinate given the Latitude,
	Longitude and Altitude.
	
	Using WGS-84.
	Input:
		lla: Array of [lat, lon, alt], [1x3] numpy array.
	Output: In tuples:
		ecef: Array of [Xe, Ye, Ze], [1x3] numpy array.
	Programmer:    Adhika Lie
	Created:    	 May 03, 2011
	Last Modified: May 03, 2011
	"""
	
	lat = lla[0]; lon = lla[1]; h = lla[2];
	
	R = earthrad(lat);
	
	x = (R[0]+h)*cos(lat)*cos(lon);
	y = (R[0]+h)*cos(lat)*sin(lon);
	z = ((1.0-ecc**2)*R[0]+h)*sin(lat);
	
	ecef = numpy.array([x, y, z]);
	return ecef;

# Function ecef2lla
def ecef2lla(ecef):
	"""
	This function calculates the Latitude, Longitude and Altitude of a
	point located on earth given the ECEF Coordinate.
	
	Using WGS-84.
	Input:
		ecef: Array of [Xe, Ye, Ze], [1x3] numpy array.
	Output: In tuples:
		lla: Array of [lat, lon, alt], [1x3] numpy array.
	Programmer:    Adhika Lie
	Created:    	 May 03, 2011
	Last Modified: May 03, 2011
	
	Reference: Jekeli, C.,"Inertial Navigation Systems With Geodetic 
	         Applications", Walter de Gruyter, New York, 2001, pp. 24
	"""
	
	x = ecef[0]; y = ecef[1]; z = ecef[2];
	lon = atan2(y,x);
	
	# Iteration
	p = sqrt(x**2+y**2);
	#Initial Guess
	lat = atan2(z,p*(1-ecc**2));
	#print 'Latitude guess is ', lat/pi*180;
	
	err = 1;
	while (fabs(err)>1e-14):
		R = earthrad(lat);
		h = p/cos(lat) - R[0];
		#print 'Altitude is ', h, 'm';
		err = atan2(z*(1+ecc**2*R[0]*sin(lat)/z),p) - lat;
		lat = lat + err;
		#print err;
	
	lla = numpy.array([lat, lon, h]);
	return lla;

# Function ecef2ned
def ecef2ned(ecef,ecef_ref):
	"""
	This function converts a vector in ecef to ned coordinate centered
	at ecef_ref.
	Input: 
	  ecef: Original vector in ECEF, 3x1
	  ecef_ref: Center for NED Coordinate, 3x1
	Output:
	  ned: Transformed vector in NED, 3x1
	Programmer:    Adhika Lie
	Created:       May 03, 2011
	Last Modified: May 21, 2011
	May 21 - Corrected syntax error of numpy.dot.
	"""
	
	C = numpy.zeros((3,3));
	ned = numpy.zeros((3,1));
	
	lla_ref = ecef2lla(ecef_ref);
	lat = lla_ref[0];
	lon = lla_ref[1];
	
	# DCM FOR ECEF TO NED
	C[0,0]=-sin(lat)*cos(lon); 
	C[0,1]=-sin(lat)*sin(lon); 
	C[0,2]=cos(lat);
	
	C[1,0]=-sin(lon); 
	C[1,1]= cos(lon); 
	C[1,2]= 0;
	
	C[2,0]=-cos(lat)*cos(lon); 
	C[2,1]=-cos(lat)*sin(lon); 
	C[2,2]=-sin(lat);
	
	ned = numpy.dot(C,ecef);
	
	return ned;

###################### MISCELLANEOUS FUNCTION ##########################
def sk(w):
	"""
	This function gives a skew symmetric matrix from a given vector w
	Input:
		w: A vector, 3x1 or 1x3
	Output:
		C: Skew Symmetric Matrix, 3x3
	Programmer:    Adhika Lie
	Created:    	 May 09, 2011
	Last Modified: May 10, 2011
	
	May 10 - Fix syntax error
	"""
	# TODO: this will not work if w is an matrix!
	C=numpy.matrix([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]]);
	
	return C;

def ortho(C):
	"""
	This function orthogonalizes a DCM by method presented in the paper
	Bar-Itzhack: "Orthogonalization Techniques of DCM" (1969, IEEE)
	Input:
		C: DCM, 3x3
	Output:
		C_ortho: Orthogonalized DCM, 3x3
	Programmer:    Adhika Lie
	Created:    	 May 10, 2011
	Last Modified: May 10, 2011
	"""
	
	e = numpy.array([1, 1, 1]);
	w1 = C[:,0]; 
	w1 = w1/numpy.linalg.norm(w1);
	
	w2 = C[:,1]; 
	w2 = w2/numpy.linalg.norm(w2);
	
	w3 = C[:,2]; 
	w3 = w3/numpy.linalg.norm(w3);
	
	while (numpy.linalg.norm(e) > 1e-15):
		w1_p = numpy.cross(w2,w3);
		w2_p = numpy.cross(w3,w1);
		w3_p = numpy.cross(w1,w2);
		
		w1_new = 0.5*(w1+w1_p);
		w1 = w1_new/numpy.linalg.norm(w1_new);
		w2_new = 0.5*(w2+w2_p);
		w2 = w2_new/numpy.linalg.norm(w2_new);
		w3_new = 0.5*(w3+w3_p);
		w3 = w3_new/numpy.linalg.norm(w3_new);
		
		e[0] = numpy.linalg.norm(w1-numpy.cross(w2,w3));
		e[1] = numpy.linalg.norm(w2-numpy.cross(w3,w1));
		e[2] = numpy.linalg.norm(w3-numpy.cross(w1,w2));
	
	w1 = w1.reshape(3,1);
	w2 = w2.reshape(3,1);
	w3 = w3.reshape(3,1);
	C_ortho = numpy.concatenate((w1,w2,w3),axis=1);
	return C_ortho
	
def wrap_pi(angles_rad):
    """
    Wrap radian angles to [-PI, +PI].
    
    Parameters
    ----------
    angle: input angles (list or np.array, units: radians)
    
    Returns
    -------
    wrapped_angle: angles wrapped to [-PI, +PI] (np.array, units: radians)
    
    Based on Adhika Lie wrapToPi.m 1/16/13 function.
    
    Note
    ----
    Care is needed at boundries.  For example, current function will wrap both
    +/- PI to -PI.  Possible future implementations may be improved to better
    handle boundry cases. 
    """
    return np.mod(angles_rad+np.pi, 2.0 * np.pi) - np.pi
########################## TEST SCRIPT #################################
#lla = numpy.array([50*pi/180, -93*pi/180, 12132.124]);
#ecef = lla2ecef(lla);
#print ecef
#my_lla = ecef2lla(ecef);

#erate = earthrate(lla[0]);
#V_NED = numpy.array([10, 10, 0]);
#rho = navrate(V_NED,lla);

#print lla
#print my_lla
#print rho
#print erate
