import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator, FormatStrFormatter
import subprocess
import io
import os

def run_executable(detector, descriptor):
    filename = f"{detector}_{descriptor}.txt"
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
    df_filtered = df.loc[(df['TTC Camera'] >= -40) & (df['TTC Camera'] <= 40)]
    any_dropped = df.shape[0] != df_filtered.shape[0]
    ttc_median_frame = df_filtered.groupby("Frame").median()
    return ttc_median_frame["TTC Camera"], any_dropped

if __name__ == "__main__":
    detector_types = ["SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"]
    descriptor_types = ["BRISK", "BRIEF", "ORB", "FREAK", "SIFT"]
    fig_camera, ax_camera = plt.subplots(figsize=(12, 8))
    ttc_median_dict = {}

    for detector in detector_types:
        for descriptor in descriptor_types:
            ttc_values = run_executable(detector, descriptor)
            if ttc_values.strip() == '0':
                continue
            ttc_median_frame, any_dropped = analyze_ttc_values(ttc_values)
            if any_dropped or ttc_median_frame.empty:
                continue
            ttc_median_dict[(detector, descriptor)] = ttc_median_frame
            constellation_label = f"{detector}_{descriptor}"
            ax_camera.plot(ttc_median_frame.index, ttc_median_frame, label=constellation_label)

    master_df = pd.concat(ttc_median_dict, axis=1)
    master_df = master_df.dropna(axis=1, how='all')
    median_values = []
    for column in master_df.columns:
        median_values.append(master_df[column].median())
    master_df.loc['Median'] = median_values
    master_df = master_df.sort_values(by='Median', axis=1, ascending=True)
    output_spreadsheet = "../results/ttc_results.xlsx"
    with pd.ExcelWriter(output_spreadsheet) as writer:
        master_df.to_excel(writer)

    ax_camera.set_xlabel("Frame")
    ax_camera.set_ylabel("TTC Camera")
    ax_camera.set_title("Camera-based TTC")
    ax_camera.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2), fancybox=True, shadow=True, ncol=5, fontsize='small')
    ax_camera.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax_camera.xaxis.set_major_formatter(FormatStrFormatter('%d'))
    plt.subplots_adjust(bottom=0.3)
    fig_camera.savefig("../results/camera_ttc_graph.png", bbox_inches='tight')
    plt.close(fig_camera)
