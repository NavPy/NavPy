import numpy as np

def angle2quat(rotAngle1,rotAngle2,rotAngle3,
                input_unit='rad',rotation_sequence='ZYX'):
    """
    Convert a sequence of rotation angles to an equivalent unit quaternion
    
    This function can take inputs in either degree or radians, and can also 
    batch process a series of rotations (e.g., time series of Euler angles).
    By default this function assumes aerospace rotation sequence but can be 
    changed using the ``rotation_sequence`` keyword argument.
    
    Parameters
    ----------
    rotAngle1, rotAngle2, rotAngle3: {(N,), (N,1), or (1,N)} angles
            They are a sequence of angles about successive axes described by 
            rotation_sequence.
    input_unit: {'rad', 'deg'}, optional. Rotation angles. Default is 'rad'.
    rotation_sequence: {'ZYX'}, optional. Rotation sequences. Default is 'ZYX'.
    
    Returns
    -------
    q0: {(N,)} array like scalar componenet of the quaternion
    qvec:{(N,3)} array like vector component of the quaternion
    
    Notes
    -----
    Convert rotation angles to unit quaternion that transfroms a vector in F1 to
    F2 according to
    
    :math: `v_q^{F2} = q^{-1} \otimes v_q^{F1} \otimes q`
    
    :math:`\otimes` indicates the quaternion multiplcation and :math:`\v_q^F`
    is a pure quaternion representation of the vector :math:`\v_q^F`. The scalar
    componenet of :math:`v_q^F` is zero.
    For aerospace sequence ('ZYX'): rotAngle1 = psi, rotAngle2 = the,
    and rotAngle3 = phi
    
    Examples
    --------
    >>> import numpy as np
    >>> from navpy import angle2quat
    >>> psi = 0
    >>> theta = np.pi/4.0
    >>> phi = np.pi/3.0
    >>> q0, qvec = angle2quat(psi,theta,phi)
    >>> q0
    0.80010314519126557
    >>> qvec
    array([ 0.46193977,  0.33141357, -0.19134172])
    
    >>> psi = [10, 20, 30]
    >>> theta = [30, 40, 50]
    >>> phi = [0, 5, 10]
    >>> q0, qvec = angle2quat(psi,theta,phi,input_unit = 'deg')
    >>> q0
    array([ 0.96225019,  0.92712639,  0.88162808])
    >>> qvec
    array([[-0.02255757,  0.25783416,  0.08418598],
           [-0.01896854,  0.34362114,  0.14832854],
           [-0.03266701,  0.4271086 ,  0.19809857]])
    """
    
    # INPUT CHECK
    rotAngle1,N1 = input_check_Nx1(rotAngle1)
    rotAngle2,N2 = input_check_Nx1(rotAngle2)
    rotAngle3,N3 = input_check_Nx1(rotAngle3)
    
    if( (N1!=N2) | (N1!=N3) | (N2!=N3) ):
        raise ValueError('Inputs are not of same dimensions')
    
    q0 = np.zeros(N1)
    qvec = np.zeros((N1,3))

    if(input_unit=='deg'):
        rotAngle1 = np.deg2rad(rotAngle1)
        rotAngle2 = np.deg2rad(rotAngle2)
        rotAngle3 = np.deg2rad(rotAngle3)
    
    rotAngle1 /= 2.0
    rotAngle2 /= 2.0
    rotAngle3 /= 2.0
    
    if(rotation_sequence=='ZYX'):
        q0[:] = np.cos(rotAngle1)*np.cos(rotAngle2)*np.cos(rotAngle3) + \
                np.sin(rotAngle1)*np.sin(rotAngle2)*np.sin(rotAngle3)

        qvec[:,0] = np.cos(rotAngle1)*np.cos(rotAngle2)*np.sin(rotAngle3) - \
            np.sin(rotAngle1)*np.sin(rotAngle2)*np.cos(rotAngle3)

        qvec[:,1] = np.cos(rotAngle1)*np.sin(rotAngle2)*np.cos(rotAngle3) + \
            np.sin(rotAngle1)*np.cos(rotAngle2)*np.sin(rotAngle3)

        qvec[:,2] = np.sin(rotAngle1)*np.cos(rotAngle2)*np.cos(rotAngle3) - \
            np.cos(rotAngle1)*np.sin(rotAngle2)*np.sin(rotAngle3)
    else:
        raise ValueError('rotation_sequence unknown')

    if(N1 == 1):
        q0 = q0[0]
        qvec = qvec.reshape(3,)
    return q0, qvec

def quat2angle(q0,qvec,output_unit='rad',rotation_sequence='ZYX'):
    """
    Convert a unit quaternion to the equivalent sequence of angles of rotation
    about the rotation_sequence axes.
    
    This function can take inputs in either degree or radians, and can also
    batch process a series of rotations (e.g., time series of quaternions).
    By default this function assumes aerospace rotation sequence but can be
    changed using the ``rotation_sequence`` keyword argument.
    
    Parameters
    ----------
    q0: {(N,), (N,1), or (1,N)} array like scalar componenet of the quaternion
    qvec:{(N,3),(3,N)} array like vector component of the quaternion
    rotation_sequence: {'ZYX'}, optional. Rotation sequences. Default is 'ZYX'.

    Returns
    -------
    rotAngle1, rotAngle2, rotAngle3: {(N,), (N,1), or (1,N)} angles
    They are a sequence of angles about successive axes described by
    rotation_sequence.
    output_unit: {'rad', 'deg'}, optional. Rotation angles. Default is 'rad'.
    
    Notes
    -----
    Convert rotation angles to unit quaternion that transfroms a vector in F1 to
    F2 according to
    
    :math: `v_q^{F2} = q^{-1} \otimes v_q^{F1} \otimes q`
    
    :math:`\otimes` indicates the quaternion multiplcation and :math:`\v_q^F`
    is a pure quaternion representation of the vector :math:`\v_q^F`. The scalar
    componenet of :math:`v_q^F` is zero.
    For aerospace sequence ('ZYX'): rotAngle1 = psi, rotAngle2 = the, 
    and rotAngle3 = phi
    
    Examples
    --------
    >>> import numpy as np
    >>> from navpy import quat2angle
    >>> q0 = 0.800103145191266
    >>> qvec = np.array([0.4619398,0.3314136,-0.1913417])
    >>> psi, theta, phi = quat2angle(q0,qvec)
    >>> psi
    1.0217702360987295e-07
    >>> theta
    0.7853982192745731
    >>> phi
    1.0471976051067484
    
    >>> psi, theta, phi = quat2angle(q0,qvec,output_unit='deg')
    >>> psi
    5.8543122160542875e-06
    >>> theta
    45.00000320152342
    >>> phi
    60.000003088824108
    
    >>> q0 = [ 0.96225019,  0.92712639,  0.88162808]
    >>> qvec = np.array([[-0.02255757,  0.25783416,  0.08418598],\
                         [-0.01896854,  0.34362114,  0.14832854],\
                         [-0.03266701,  0.4271086 ,  0.19809857]])
    >>> psi, theta, phi = quat2angle(q0,qvec,output_unit='deg')
    >>> psi
    array([  9.99999941,  19.99999997,  29.9999993 ])
    >>> theta
    array([ 30.00000008,  39.99999971,  50.00000025])
    >>> phi
    array([ -6.06200867e-07,   5.00000036e+00,   1.00000001e+01])
    """
    q0, N0 = input_check_Nx1(q0)
    qvec, Nvec = input_check_Nx3(qvec)

    if(N0!=Nvec):
        raise ValueError('Inputs are not of same dimensions')
    
    q1 = qvec[:,0]
    q2 = qvec[:,1]
    q3 = qvec[:,2]

    rotAngle1 = np.zeros(N0)
    rotAngle2 = np.zeros(N0)
    rotAngle3 = np.zeros(N0)

    if(rotation_sequence=='ZYX'):
        m11 = 2*q0**2 + 2*q1**2 - 1
        m12 = 2*q1*q2 + 2*q0*q3
        m13 = 2*q1*q3 - 2*q0*q2
        m23 = 2*q2*q3 + 2*q0*q1
        m33 = 2*q0**2 + 2*q3**2 - 1

        rotAngle1 = np.arctan2(m12,m11)
        rotAngle2 = np.arcsin(-m13)
        rotAngle3 = np.arctan2(m23,m33)
    else:
        raise ValueError('rotation_sequence unknown')

    if(N0 == 1):
        rotAngle1 = rotAngle1[0]
        rotAngle2 = rotAngle2[0]
        rotAngle3 = rotAngle3[0]
    if(output_unit=='deg'):
        rotAngle1 = np.rad2deg(rotAngle1)
        rotAngle2 = np.rad2deg(rotAngle2)
        rotAngle3 = np.rad2deg(rotAngle3)

    return rotAngle1, rotAngle2, rotAngle3

def quat2dcm(q0,qvec,rotation_sequence='ZYX',output_type='ndarray'):
    """
    Convert a single unit quaternion to one DCM
    
    Parameters
    ----------
    q0: {(N,), (N,1), or (1,N)} array like scalar componenet of the quaternion
    qvec:{(N,3),(3,N)} array like vector component of the quaternion
    rotation_sequence: {'ZYX'}, optional. Rotation sequences. Default is 'ZYX'.
    output_type: {'ndarray','matrix'}, optional. Output is either numpy array
            (default) or numpy matrix.
            
    Returns
    -------
    C_N2B: direction consine matrix that rotates the vector from the first frame
        to the second frame according to the specified rotation_sequence.
        
    Examples
    --------
    >>> import numpy as np
    >>> from navpy import quat2dcm
    >>> q0 = 1
    >>> qvec = [0, 0, 0]
    >>> C = quat2dcm(q0,qvec)
    >>> C
    array([[ 1.,  0.,  0.],
           [ 0.,  1.,  0.],
           [ 0.,  0.,  1.]])
    
    >>> q0 = 0.9811
    >>> qvec = np.array([-0.0151, 0.0858, 0.1730])
    >>> C = quat2dcm(q0,qvec,output_type='matrix')
    >>> C
    matrix([[  9.25570440e-01,   3.36869440e-01,  -1.73581360e-01],
            [ -3.42051760e-01,   9.39837700e-01,   5.75800000e-05],
            [  1.63132160e-01,   5.93160200e-02,   9.84972420e-01]])
    """
    # Input check
    q0,N0 = input_check_Nx1(q0)
    qvec,Nvec = input_check_Nx3(qvec)

    if((N0!=1) | (Nvec!=1)):
        raise ValueError('Can only process 1 quaternion')

    q1 = qvec[:,0]
    q2 = qvec[:,1]
    q3 = qvec[:,2]
    if(rotation_sequence=='ZYX'):
        C_N2B = np.zeros((3,3))

        C_N2B[0,0] =  2*q0**2 - 1 + 2*q1**2
        C_N2B[1,1] =  2*q0**2 - 1 + 2*q2**2
        C_N2B[2,2] =  2*q0**2 - 1 + 2*q3**2

        C_N2B[0,1] = 2*q1*q2 + 2*q0*q3
        C_N2B[0,2] = 2*q1*q3 - 2*q0*q2

        C_N2B[1,0] = 2*q1*q2 - 2*q0*q3
        C_N2B[1,2] = 2*q2*q3 + 2*q0*q1

        C_N2B[2,0] = 2*q1*q3 + 2*q0*q2
        C_N2B[2,1] = 2*q2*q3 - 2*q0*q1
    else:
        raise ValueError('rotation_sequence unknown')

    if(output_type=='matrix'):
        C_N2B = np.asmatrix(C_N2B)

    return C_N2B

def dcm2quat(C,rotation_sequence='ZYX'):
    """
    Convert a DCM to a unit quaternion
    
    Parameters
    ----------
    C: direction consine matrix that rotates the vector from the first frame
    to the second frame according to the specified rotation_sequence.
    rotation_sequence: {'ZYX'}, optional. Rotation sequences. Default is 'ZYX'.
    
    Return
    ------
    q0: {(N,)} array like scalar componenet of the quaternion
    qvec:{(N,3)} array like vector component of the quaternion

    Examples
    --------
    >>> import numpy as np
    >>> from navpy import dcm2quat
    >>> C = np.array([[  9.25570440e-01,   3.36869440e-01,  -1.73581360e-01],
                      [ -3.42051760e-01,   9.39837700e-01,   5.75800000e-05],
                      [  1.63132160e-01,   5.93160200e-02,   9.84972420e-01]])
    >>> q0,qvec = dcm2quat(C)
    >>> q0
    0.98111933015306552
    >>> qvec
    array([-0.0150997 ,  0.08579831,  0.17299659])
    """
    
    if(C.shape[0]!=C.shape[1]):
        raise ValueError('Input is not a square matrix')
    if(C.shape[0]!=3):
        raise ValueError('Input needs to be a 3x3 array or matrix')

    qvec = np.zeros(3)
    q0 = 0.5*np.sqrt(C[0,0]+C[1,1]+C[2,2]+1)
    qvec[0] = (C[1,2]-C[2,1])/(4*q0)
    qvec[1] = (C[2,0]-C[0,2])/(4*q0)
    qvec[2] = (C[0,1]-C[1,0])/(4*q0)

    return q0,qvec

def input_check_Nx1(x):
    x = np.atleast_1d(x)
    theSize = np.shape(x)

    if(len(theSize)>1):
        #1. Input must be of size N x 1
        if ((theSize[0]!=1) & (theSize[1]!=1)):
            raise ValueError('Not an N x 1 array')
        #2. Make it into a 1-D array
        x = x.reshape(np.size(x))
    else:
        x = x[0]
    return x,np.size(x)

def input_check_Nx3(x):
    x = np.atleast_2d(x)
    theSize = np.shape(x)
    
    if(len(theSize)>1):
        #1. Input must be of size N x 3
        if ((theSize[0]!=3) & (theSize[1]!=3)):
            raise ValueError('Not a N x 3 array')
        #2. Make it into a Nx3 array
        if (theSize[1]!=3):
            x = x.T
    
    return x,x.shape[0]