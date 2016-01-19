"""
Utilities Functions.

Copyright (c) 2014 NavPy Developers. All rights reserved.
Use of this source code is governed by a BSD-style license that can be found in
LICENSE.txt
"""

import numpy as _np
import sys

def input_check_Nx1(x):
    """
    Check x to be of dimension Nx1 and reshape it as a 1-D array

    Adhika Lie
    """
    x = _np.atleast_1d(x)
    theSize = _np.shape(x)

    if(len(theSize) > 1):
        # 1. Input must be of size N x 1
        if ((theSize[0] != 1) & (theSize[1] != 1)):
            raise ValueError('Not an N x 1 array')
        # 2. Make it into a 1-D array
        x = x.reshape(_np.size(x))
    elif (theSize[0] == 1):
        x = x[0]

    return x, _np.size(x)


def input_check_Nx3(x):
    """
    Check x to be of dimension Nx3

    Adhika Lie
    """
    x = _np.atleast_2d(x)
    theSize = _np.shape(x)

    if(len(theSize) > 1):
        # 1. Input must be of size N x 3
        if ((theSize[0] != 3) & (theSize[1] != 3)):
            raise ValueError('Not a N x 3 array')
        # 2. Make it into a Nx3 array
        if (theSize[1] != 3):
            x = x.T
        N = x.shape[0]
        # 3. If N == 1, make it into a 1-D array
        if (x.shape[0] == 1):
            x = x.reshape(x.shape[1])

    return x, N


def input_check_Nx3x3(x):
    """
    Check x to be of dimension Nx3x3

    Jacob Niehus
    """
    theSize = _np.shape(x)
    N = 1

    if(len(theSize) > 2):
        # 1. Input must be of size N x 3
        if (3, 3) not in (theSize[:2], theSize[-2:]):
            raise ValueError('Not a N x 3 x 3 array')
        # 2. Make it into a Nx3x3 array
        if (theSize[1:] != (3, 3)):
            x = _np.rollaxis(x, -1)
        N = x.shape[0]
        # 3. If N == 2, make it into a 2-D array
        if (x.shape[0] == 1):
            x = x[0]
    elif(theSize != (3, 3)):
        raise ValueError('Not a 3 x 3 array')

    return x, N


def loadtxt2dic(filename):
    """
    Loads text file of key:value pairs into a dictionary.
    Usage notes:
    -Lines begining with '#' are treated as comments and skipped.
    -Blank lines are also skipped
    -Keys and values should be separated by '=' or ':', extra spaces are fine.
    -A matrix/scalar are stored floats ONLY if the text has a decimal

    Hamid M. (original)
    Adhika Lie
    """
    fid = open(filename, 'r')
    param = {}
    prev_line = ''

    for line in fid:
        # Remove Extra Spaces
        line = prev_line + line.strip()
        print(line)
        # Skip lines beginning with # or blank
        # Note: Python treats '' as False
        if(line.startswith('#') or line.startswith('\n') or (not line)):
            continue

        # If line ends with a comma, it continues to the next line.
        if(line.endswith(',')):
            prev_line = line.strip()
            continue
        else:
            prev_line = ''

        # Split item
        item = line.split('#', 1)[0].strip()  # May have comment after the line
        item = item.replace(':', ' ').replace('=', ' ').split(None, 1)

        if(len(item) == 0):
            continue

        try:
            param[item[0]] = eval(item[1])

            if(type(eval(item[1])) == list):
                param[item[0]] = _np.array(eval(item[1]))

        except NameError:
            param[item[0]] = item[1]

    fid.close()
    return param


def ask_ok(prompt, retries=4, complaint='Yes or no, please!'):
    """
    Prompt user for for 'yes' or 'no' response
    Taken from Python Documentation with small modifications
    http://docs.python.org/tutorial/controlflow.html

    Example:
    >>> ask_ok('Do you really want to quit?')

    Hamid M. May 2012
    """
    while True:

        ok = raw_input(prompt).lower()

        if ok in ('y', 'ye', 'yes', '1'):
            return True
        if ok in ('n', 'no', 'nop', 'nope', '0'):
            return False
        retries = retries - 1
        if retries < 0:
            raise IOError('refusenik user')
        print(complaint)


def status_update(i, drl, message, bar_length=20):
    """
    To create progress bar + short error message

    Parameters
    ----------
    i is the counter of where you are in the "count"
    drl is the total data record length
    message is your message line (keep it short!)

    Adhika Lie
    """
    percent = float(i) / drl
    hashes = '#' * int(round(percent * bar_length))
    spaces = ' ' * (bar_length - len(hashes))

    mmsg = message
    if(len(mmsg) > 0):
        if(mmsg[-1] != '\n'):
            mmsg = mmsg + '\n'

    sys.stdout.write("\r[%s] %3d%% :: %s" %
                     (hashes + spaces, int(round(percent * 100)), mmsg))
    sys.stdout.flush()
