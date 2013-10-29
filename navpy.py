import numpy as np

def angle2quat(rotAngle1,rotAngle2,rotAngle3,
                input_unit='rad',rotation_sequence='ZYX'):
    """
    
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

    return q0, qvec

def quat2angle(q0,qvec,output_unit='rad',rotation_sequence='ZYX'):
    """
    
    """


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
