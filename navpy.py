import numpy as np

def eul2quat(rotAngle1,rotAngle2,rotAngle3,
                input_unit='rad',rotation_sequence='ZYX'):
    """
    
    """
    """
    # INPUT CHECK
    rotAngle1 = np.array(rotAngle1)
    rotAngle2 = np.array(rotAngle2)
    rotAngle3 = np.array(rotAngle3)
    if(len(rotAngle1.shape)==0):
        rotAngle1.reshape(1,)
    if(len(rotAngle2.shape)==0):
        rotAngle2.reshape(1,)
    if(len(rotAngle3.shape)==0):
        rotAngle3.reshape(1,)

    if(len(rotAngle1.shape)==2)

    rotAngle1.shape[0]
    """
       
    if(input_unit=='deg'):
        rotAngle1 = np.deg2rad(rotAngle1)
        rotAngle2 = np.deg2rad(rotAngle2)
        rotAngle3 = np.deg2rad(rotAngle3)
    
    rotAngle1 /= 2.0
    rotAngle2 /= 2.0
    rotAngle3 /= 2.0
    
    if(rotation_sequence=='ZYX'):
        q0 = np.cos(rotAngle1)*np.cos(rotAngle2)*np.cos(rotAngle3) + \
                np.sin(rotAngle1)*np.sin(rotAngle2)*np.sin(rotAngle3)

        qvec = np.zeros(3)
        qvec[0] = np.cos(rotAngle1)*np.cos(rotAngle2)*np.sin(rotAngle3) - \
            np.sin(rotAngle1)*np.sin(rotAngle2)*np.cos(rotAngle3)

        qvec[1] = np.cos(rotAngle1)*np.sin(rotAngle2)*np.cos(rotAngle3) + \
            np.sin(rotAngle1)*np.cos(rotAngle2)*np.sin(rotAngle3)

        qvec[2] = np.sin(rotAngle1)*np.cos(rotAngle2)*np.cos(rotAngle3) - \
            np.cos(rotAngle1)*np.sin(rotAngle2)*np.sin(rotAngle3)

    return q0, qvec