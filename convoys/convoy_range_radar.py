import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
from collections import deque
import os
import sys
sys.path.append('/home/user/repos/perception')
from robotics_utils.math.polar_conversion import sph2cart
from robotics_utils.video.frames_to_video import frames_to_video


def find_best_match(df_curr: pd.DataFrame, curr_best_queue: deque[pd.Series]) -> pd.Series:
    df_best = pd.DataFrame(curr_best_queue)
    range_best = df_best['Range[m]'].values
    range_curr = df_curr['Range[m]'].values
    azimuth_best = df_best['Azimuth_Angle_rad[rad]'].values
    azimuth_curr = df_curr['Azimuth_Angle_rad[rad]'].values
    elevation_best = df_best['Elevation_Angle_rad[rad]'].values
    elevation_curr = df_curr['Elevation_Angle_rad[rad]'].values
    rcs_best = df_best['RCS[dB]'].values
    rcs_curr = df_curr['RCS[dB]'].values
    snr_best = df_best['SNR[dB]'].values
    snr_curr = df_curr['SNR[dB]'].values
    w1, w2, w3, w4, w5 = 0.333, 0.333, 0.333, 0.0, 0.0
    diff = np.sqrt(
        w1 * (range_best.reshape(-1,1) - range_curr.reshape(1,-1)) ** 2 +
        w2 * (azimuth_best.reshape(-1,1) - azimuth_curr.reshape(1,-1)) ** 2 +
        w3 * (elevation_best.reshape(-1,1) - elevation_curr.reshape(1,-1)) ** 2 
        + w4 * (rcs_best.reshape(-1,1) - rcs_curr.reshape(1,-1)) ** 2 +
        w5 * (snr_best.reshape(-1,1) - snr_curr.reshape(1,-1)) ** 2
    )
    diff = pd.DataFrame(diff, index=df_best.index, columns=df_curr.index)
    idx_min = diff.min().idxmin()
    return df_curr.loc[idx_min]


def read_radar_data(csv_file: str) -> pd.DataFrame:
    df = pd.read_excel(csv_file, sheet_name='base')
    df.columns = [col.split('.', 1)[-1].replace(' ','') for col in df.columns]
    df = df.drop('', axis=1)
    df['SNR[dB]'] = df['Signal_Level[dB]'] - df['Noise[dB]']
    return df


def calc_range_radar(df: pd.DataFrame) -> pd.DataFrame:
    cycle_counts = sorted(df['CycleCount'].unique())
    curr_best_queue = deque(maxlen=5)
    best_route = []
    for idx, cycle_count in enumerate(cycle_counts):
        df_curr = df.loc[df['CycleCount'] == cycle_count]
        if idx == 0 or df_curr['SNR[dB]'].max() >= 50.0:
            idx_max = df_curr['SNR[dB]'].idxmax()
            curr_best_queue.append(df.loc[idx_max])
            best_route.append(df.loc[idx_max])
            continue
        best_curr = find_best_match(df_curr, curr_best_queue)
        curr_best_queue.append(best_curr)
        best_route.append(best_curr)
    return pd.DataFrame(best_route)


def plot_graphs(best_route: pd.DataFrame) -> None:
    best_range = best_route['Range[m]']
    best_azimuth = best_route['Azimuth_Angle_rad[rad]']
    best_elevation = best_route['Elevation_Angle_rad[rad]']
    fig, (ax1, ax2) = plt.subplots(2, 1)
    ax1.plot(best_range)
    ax1.set_title('Range')
    ax1.set_ylabel('range[m]')
    ax1.grid(True)
    ax2.plot(best_azimuth)
    ax2.set_title('Azimuth')
    ax2.set_ylabel('azimuth[rad]')
    ax2.grid(True)

    plt.tight_layout()
    plt.show()
    

def write_video(df: pd.DataFrame, best_route: pd.DataFrame, src_dir: str):
    # src_dir = '/home/user/work/exp/convoy/radar_xy/'
    if not os.path.exists(src_dir):
        os.makedirs(src_dir)
    best_range = best_route['Range[m]']
    best_azimuth = best_route['Azimuth_Angle_rad[rad]']
    best_elevation = best_route['Elevation_Angle_rad[rad]']
    
    best_x, best_y, best_z = sph2cart(best_azimuth, best_elevation, best_range)

    cycle_counts = sorted(df['CycleCount'].unique())
    for idx, cycle_count in enumerate(cycle_counts):
        df_curr = df.loc[df['CycleCount'] == cycle_count]
        curr_range = df_curr['Range[m]']
        curr_azimuth = df_curr['Azimuth_Angle_rad[rad]']
        curr_elevation = df_curr['Elevation_Angle_rad[rad]']

        curr_x, curr_y, curr_z = sph2cart(curr_azimuth, curr_elevation, curr_range)
        
        plt.figure()
        plt.scatter(curr_x, curr_y)
        plt.scatter(best_x.iloc[idx], best_y.iloc[idx])
        plt.xlim(0, 60)
        plt.ylim(-20, 20)
        plt.grid()
        plt.savefig(f'{src_dir}/{int(idx):04}')
    
    frames_to_video(src_dir, f'{src_dir}/radar_tracker.mp4', extension='png', is_resize=False)
    

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--in_csv', help='Path to the input daya in cvs file.',
                            default='/home/user/repos/perception/convoys/T1-radar-analasys-220524.xlsx')
    parser.add_argument('--out_csv', help='Path to the output daya in cvs file.',
                            default='/home/user/repos/perception/convoys/T1-radar-analasys-220524_best_route.xlsx')
    parser.add_argument('--src_dir', help='path to directory for writing video', default='/home/user/work/exp/convoy/radar_xy/')
    return parser.parse_args()

def main():
    args = parse_args()
    # csv_file = '/home/user/repos/perception/convoys/T1-radar-analasys-220524.xlsx'
    df = read_radar_data(args.in_csv)
    best_route = calc_range_radar(df)
    # out_csv_file = '/home/user/repos/perception/convoys/T1-radar-analasys-220524_best_route.xlsx'
    best_route.to_csv(args.out_csv)
    # best_route = pd.read_csv(out_csv_file)
    # plot_graphs(best_route)

    write_video(df, best_route, args.src_dir)


if __name__ == '__main__':
    main()