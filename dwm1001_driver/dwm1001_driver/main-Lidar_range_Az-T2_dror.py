import numpy as np
import os
import open3d as o3d
import pandas as pd
import time
from scapy.all import rdpcap


CURRENT_DIR = os.path.dirname(__file__)
#FRAMES_DIR = os.path.join(CURRENT_DIR, "test")
FRAMES_DIR = os.path.abspath(os.path.join(
    CURRENT_DIR,
    "..",
    "2026-01-20_hadid_test_lidar_static",
    "static_13m_csv",
 #   "dynamic1",
))

MIN_INTENSITY = 101
MAX_X = 20
MIN_X = -20
MAX_Y = 30
MIN_Y = 2
MAX_Z = 5
MIN_Z = 0.2

EPS_VALUE = 1.5
MIN_CLUSTER_AMOUNT = 20

RESULT_FILE_NAME = "results.csv"


def get_azimuth_and_distance(x, y):
    """
    Calculate the azimuth and distance to the middle of the largest cluster.
    """
    # Calculate centroid of the largest cluster
    distance = np.linalg.norm(np.array([x, y, 0]))
    degree = np.degrees(np.arctan2(x, y))

    return degree, distance


def filter_irrelevant_points(frame):
    return frame.query(
                        f"intensity >= {MIN_INTENSITY} and \
                        intensity != 100 and \
                        {MAX_X} >= X >= {MIN_X} and \
                        {MAX_Y} >= Y >= {MIN_Y} and \
                        {MAX_Z} >= Z >= {MIN_Z}")


def remove_secondaries_clusters(frame):
    o3d_space = o3d.geometry.PointCloud()
    o3d_space.points = o3d.utility.Vector3dVector(frame[['X', 'Y', 'Z']].values)
    #from IPython import embed; embed()

    if len(o3d_space.points):
        frame["cluster"] = np.array(o3d_space.cluster_dbscan(eps=EPS_VALUE, min_points=MIN_CLUSTER_AMOUNT, print_progress=False))
        filtered_frame = frame[frame['cluster'] == 0]
        # print("frame-1=", filtered_frame)
        if not filtered_frame.empty:
            most_common_cluster = filtered_frame['cluster'].value_counts().idxmax()
            #print("frame-2=", frame)

            return frame[(frame['cluster'] == most_common_cluster)]

    frame["cluster"] = 0

    return frame


def get_center_point(frame):
    board_points_cloud = o3d.geometry.PointCloud()
    board_points_cloud.points = o3d.utility.Vector3dVector(frame[['X', 'Y', 'Z']].values)

    r = board_points_cloud.get_center()
  #  print("r=", r)
    return r

def convert_names(frame):
  #  frame = frame.rename(columns={'Points_m_XYZ:0': 'X', 'Points_m_XYZ:1': 'Y', 'Points_m_XYZ:2': 'Z'})
    frame['X'] = frame['X'].astype(float)
    frame['Y'] = frame['Y'].astype(float)
    frame['Z'] = frame['Z'].astype(float)
    return frame

def main():     
    # pcap test ######
    #Read the PCAP file
    #packets = rdpcap('static_6.2.mpcap');

# # Iterate through packets
    #for packet in packets:
#            print(f"{packet}");
         





    # end pcap test ######


    # print("Lidar Range and Azimuth Calculation Started")
    # Create a list to store frames
    frames_centers = pd.DataFrame({
    "X": [],
    "Y": [],
    "degree": [],
    "distance": [],
    "time": []
    })

    # Iterate over each text file

    for file_name in sorted(os.listdir(FRAMES_DIR)):
        
        if file_name.endswith(".csv"):
            print(f"Working {file_name}")
            # Load points
            file_path = os.path.join(FRAMES_DIR, file_name)
            frame = pd.read_csv(file_path)
            frame = convert_names(frame)

            timestamp = frame['timestamp'].median() / 1000
            frame = filter_irrelevant_points(frame)
            # print("frame-0=", frame)
            frame = remove_secondaries_clusters(frame)
            # print("frame-3=", frame)
            center_x, center_y, center_z = get_center_point(frame)
            # print("center point=", center_x, center_y, center_z)
            center_degree, center_distance = get_azimuth_and_distance(x=center_x, y=center_y)
            
            if frame['timestamp'].median() == np.nan:
                timestamp = frame['timestamp'].median() / 1000

            frames_centers.loc[file_name] = {
                "X": center_x,
                "Y": center_y,
                "degree": center_degree,
                "distance": center_distance,
                "time": timestamp
            }
    frames_centers.to_csv(RESULT_FILE_NAME)


if __name__ == "__main__":
    main()
