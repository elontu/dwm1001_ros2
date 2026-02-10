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


def convoy_range(csv_file: str, d: float, l: float, sync_index: int=0) -> pd.DataFrame:

    cols = ['timestamp', 'num', 'anchor1', 'anchodr1_id', 'x1', 'y1', 'z1', 'r1', 'anchor2', 'anchodr2_id', 'x2', 'y2', 'z2', 'r2']
    df_left = pd.read_excel(csv_file, sheet_name='left', header=None, index_col=None, names=cols, usecols='A,H:U')
    df_right = pd.read_excel(csv_file, sheet_name='right', header=None, index_col=None, names=cols, usecols='A,H:U')
    
    r1 = df_left['r1']
    r2 = df_left['r2']
    r3 = df_right['r1'].iloc[1:]
    r4 = df_right['r2'].iloc[1:]

    r3.index = [i - sync_index for i in r3.index]
    r4.index = [i - sync_index for i in r4.index]

    # Analytic approach

    # x1, y1 = get_coord(r1, r2, d)
    # x2, y2 = get_coord(r3, r4, d)

    # x_m = (x1 + x2) / 2
    # y_m = (y1 + y2) / 2

    # r, a = get_range_angle(x_m, y_m, d, l)

    # df = pd.concat([x_m, y_m, r, a], axis=1)
    # df.columns = ['x', 'y', 'range[m]', 'angle[deg]']

    # Geometric approach

    m1 = median_theorem(r1, r2, d)
    m2 = median_theorem(r3, r4, d)
    m = median_theorem(m1, m2, d)

    alpha = np.pi/2 - cosine_law_angle(m, d/2, m2)

    # not need to calculate r_geo, a_tag, a_geo for lidar

    r_geo = cosine_law_line(l, m, alpha)
    a_tag = sine_law(m, r_geo, alpha)
    # a_geo = np.rad2deg(np.sign(a_tag) * (np.pi - np.abs(a_tag)))
    a_geo = np.rad2deg(sine_law(m, r_geo, alpha) - np.pi/2)
    # a_geo = np.rad2deg(a_tag)

    df = pd.concat([x_m, y_m, r, a, r_geo, a_geo], axis=1)
    df.columns = ['x', 'y', 'range[m]', 'angle[deg]', 'range_geo[m]', 'angle_geo[deg]']

    return df



def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--in_csv', help='Path to the input daya in cvs file.',
                            # default='/home/user/repos/perception/convoys/T1-08-05-24.xlsx')
                            default='/home/user/repos/convoys/test_0508-ohad/test-050824-UWB/T2-05-08-24.xlsx')
    parser.add_argument('--out_csv', help='Path to the output daya in cvs file.',
                            default='/home/user/repos/perception/convoys/T2-05-08-24_results.xlsx')
    parser.add_argument('--d', help='distance between 2 anchors' ,default=2.12)
    parser.add_argument('--l', help='distance between Lidar and UWB' ,default=2)
    parser.add_argument('--sync_index', help='the index when the synchronization between left and right is occured.' ,default=1)
    return parser.parse_args()


def main():
    args = parse_args()
    # csv_file = '/home/user/repos/perception/convoys/T1-08-05-24.xlsx'
    # d = 2.12
    # out_csv_file = '/home/user/repos/perception/convoys/T1-08-05-24_my_results.xlsx'
    df = convoy_range(args.in_csv, args.d, args.l, args.sync_index)
    df.to_csv(args.out_csv, index=False)


if __name__ == '__main__':
    main()