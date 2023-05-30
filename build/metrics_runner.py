import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator, FormatStrFormatter
import subprocess
import io
import os
import pdb

def run_executable(detector, descriptor):
    filename = f"./tmp/{detector}_{descriptor}.txt"
    if os.path.exists(filename):
        with open(filename, 'r') as f:
            result = f.read()
    else:
        executable_name = "./3D_object_tracking"
        result = subprocess.run([executable_name, detector, descriptor], capture_output=True, text=True).stdout
        with open(filename, 'w') as f:
            f.write(result)
    return result

def analyze_ttc_values(ttc_values):
    csv_string_io = io.StringIO(ttc_values)
    df = pd.read_csv(csv_string_io, header=None, names=["Frame", "TTC Lidar", "TTC Camera"])
    df_filtered = df.loc[(df['TTC Camera'] >= -50) & (df['TTC Camera'] <= 50)]
    any_dropped = df.shape[0] != df_filtered.shape[0]
    return df_filtered, any_dropped

if __name__ == "__main__":
    detector_types = ["SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"]
    descriptor_types = ["BRISK", "BRIEF", "ORB", "FREAK", "SIFT"]
    fig_camera, ax_camera = plt.subplots(figsize=(12, 8))
    ttc_dict = {}
    ttc_lidar_dict = {}

    for detector in detector_types:
        for descriptor in descriptor_types:
            ttc_values = run_executable(detector, descriptor)
            if ttc_values.strip() == '0':
                continue
            ttc_df, any_dropped = analyze_ttc_values(ttc_values)
            if any_dropped or ttc_df.empty:
                print(f"Skipping {detector} {descriptor}")
                continue
            if len(ttc_lidar_dict) == 0:
                ttc_lidar_dict = ttc_df["TTC Lidar"]
            ttc_dict[(detector, descriptor)] = ttc_df["TTC Camera"]

    master_df = pd.concat(ttc_dict, axis=1)
    master_df = master_df.dropna(axis=1, how='all')

    corr_values = []
    for column in master_df.columns:
        corr_values.append(np.corrcoef(master_df[column], ttc_lidar_dict)[0, 1])
    master_df.loc['Correlation'] = corr_values

    master_df = master_df.sort_values(by='Correlation', axis=1, ascending=False)
    output_spreadsheet = "../results/ttc_results.xlsx"
    with pd.ExcelWriter(output_spreadsheet) as writer:
        master_df.to_excel(writer)

    master_df = master_df.iloc[:, :5]
    master_df.insert(0, "TTC Lidar", ttc_lidar_dict)
    master_df = master_df.drop(master_df.tail(1).index)

    for column in master_df.columns:
        label = ' '.join(column).strip()
        if label == "TTC Lidar":
            ax_camera.plot(master_df[column], label=label, linewidth=3)
        else:
            ax_camera.plot(master_df[column], label=label)

    ax_camera.set_xlabel("Frame")
    ax_camera.set_ylabel("TTC (s)")
    ax_camera.set_title("Camera-based TTC")
    ax_camera.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2), fancybox=True, shadow=True, ncol=5, fontsize='small')
    ax_camera.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax_camera.xaxis.set_major_formatter(FormatStrFormatter('%d'))
    plt.subplots_adjust(bottom=0.3)
    fig_camera.savefig("../results/camera_ttc_graph.png", bbox_inches='tight')
    plt.close(fig_camera)
