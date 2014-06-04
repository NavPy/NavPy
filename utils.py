import numpy as _np

def input_check_Nx1(x):
    x = _np.atleast_1d(x)
    theSize = _np.shape(x)

    if(len(theSize)>1):
        #1. Input must be of size N x 1
        if ((theSize[0]!=1) & (theSize[1]!=1)):
            raise ValueError('Not an N x 1 array')
        #2. Make it into a 1-D array
        x = x.reshape(_np.size(x))
    elif (theSize[0]==1):
        x = x[0]
    
    return x,_np.size(x)

def input_check_Nx3(x):
    x = _np.atleast_2d(x)
    theSize = _np.shape(x)
    
    if(len(theSize)>1):
        #1. Input must be of size N x 3
        if ((theSize[0]!=3) & (theSize[1]!=3)):
            raise ValueError('Not a N x 3 array')
        #2. Make it into a Nx3 array
        if (theSize[1]!=3):
            x = x.T
        N = x.shape[0]
        #3. If N == 1, make it into a 1-D array
        if (x.shape[0]==1):
            x = x.reshape(x.shape[1])

    return x,N
