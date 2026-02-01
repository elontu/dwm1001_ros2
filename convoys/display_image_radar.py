import pandas as pd
import numpy as np
import argparse

'''
The equations for the convoy range is defined by:
R1^2 = x1^2 + y1^2
R2^2 = (x1-d)^2 +y1^2
-----------------------
The solutions are:
x1 = (R1^2 - R2^2 + d^2) / 2d
y1 = sqrt(R1^2-x1^2)
-----------------------
And the range and the angle are:
range = sqrt((x1-d/2)^2 + y1^2)
angle = pi/2 - arctan(y1 / (x1-d/2))
'''


def median_theorem(a, b, d):
    return np.sqrt((a**2 + b**2 - (d**2)/2)/2)

def cosine_law_angle(a, b, c):
    return np.arccos((a**2 + b**2 - c**2)/(2*a*b))

def cosine_law_line(a, b, alpha):
    return np.sqrt(a**2 + b**2 - 2*a*b*np.cos(alpha))

def sine_law(a, b, beta):
    return np.arcsin((a*np.sin(beta))/b)

def get_coord(r1, r2, d):
    assert d != 0, "d must not be 0."
    x = abs((r1**2 - r2**2 + d**2)/(2*d))
    y = np.sqrt(r1**2 - x**2)
    return x, y


def get_range_angle(x, y, d, l):
    init_r = np.sqrt((x - d/2) ** 2 + y ** 2)
    init_a = np.arctan(abs(x-d/2) / y)
    r = cosine_law_line(init_r, l, init_a)
    a = np.rad2deg(np.arctan(abs(x-d/2) / (y-l)))
    return r, a

def convoy_range_azimuth(r1, r2, r3, r4, d1, d2):
    '''
    Calculate the range and azimuth between two UWB tags mounted on two vehicles in a convoy setup.
    Each vehicle has two anchors separated by distance d1 and d2.
    Args:
        r1, r2: Distances from the follower vehicle's anchors to the left target (UWB tag) on leader
        r3, r4: Distances from the follower vehicle's anchors to the right target (UWB tag) on leader
        d1: Distance between the two anchors leader vehicle
        d2: Distance between the two anchors follower vehicle
    Returns:
        range: Estimated range between the two centers of the vehicles
        azimuth: Estimated azimuth angle (radians) from follower to leader
    '''
    m1 = median_theorem(r1, r2, d1)
    m2 = median_theorem(r3, r4, d1)
    range = median_theorem(m1, m2, d2)

    azimuth = np.pi/2 - cosine_law_angle(range, d2/2, m2) # if positive leader turn right, else left

    return range, azimuth
