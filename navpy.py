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


def input_check_Nx1(x):
    x = np.atleast_1d(x)
    theSize = np.shape(x)

    if(len(theSize)>1):
        #1. Input must be of size N x 1
        if ((theSize[0]!=1) & (theSize[1]!=1)):
            raise ValueError('Not an N x 1 array')
        #2. Make it into a 1-D array
        x = x.reshape(np.size(x))

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