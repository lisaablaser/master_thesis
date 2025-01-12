import pandas as pd
import numpy as np
import os

def aggregate_by_iteration(folder_path, output_file):

    csv_files = [
        os.path.join(folder_path, f)
        for f in os.listdir(folder_path)
        if f.endswith(".csv")
    ]

    if not csv_files:
        print("No CSV files found in the folder.")
        return

    dataframes_ffill = []
    dataframes_original = []
    max_len = -1

    for file in csv_files:
        print(f"Loading {file}...")
        df = pd.read_csv(file)
        if df["Iteration"].duplicated().any():
            print(f"Duplicate iterations detected in {file}. Removing duplicates.")
            df = df.drop_duplicates(subset="Iteration", keep="first")

        dataframes_original.append(df)
        dataframes_ffill.append(df)
        l = df.shape[0]
        max_len = max(max_len, l)

    for i, df in enumerate(dataframes_ffill):
        dataframes_ffill[i] = (
            df.set_index("Iteration")
            .reindex(range(max_len))
            .fillna(method="ffill")
            .reset_index()
        )

    combined_data_ffill = pd.concat(dataframes_ffill, ignore_index=True)
    combined_data_original = pd.concat(dataframes_original, ignore_index=True)

    aggregated_1 = (
        combined_data_ffill.groupby("Iteration")
        .agg(
            {
                "Progress": ["mean", "std"],
                "TrajectoryLength": ["mean", "std"],
            }
        )
        .reset_index()
    )
    aggregated_1.columns = [
        "_".join(col).strip() if col[1] else col[0]
        for col in aggregated_1.columns.values
    ]

    aggregated_2 = (
        combined_data_original.groupby("Iteration")
        .agg(
            {
                "Time": ["mean", "std"],
                "GenerateTime": ["mean", "std"],
                "EvalTime": ["mean", "std"],
                "EvalAvgTime": ["mean", "std"],
                "SelectTime": ["mean", "std"],
                "Attempts": ["mean", "std"],
                "Candidates": ["mean", "std"],
                "EstimatedGain": ["mean", "std"],
                "UtilityScore": ["mean", "std"],
                "NBVTime": ["mean", "std"],
                "Planner": ["mean", "std"],
                "MoveTime": ["mean", "std"],
                "MapUpdateTime": ["mean", "std"],
                "Gain": ["mean", "std"],
                "ClusterTime": ["mean", "std"],
                "NbvMemory": ["mean", "std"],
            }
        )
        .reset_index()
    )
    aggregated_2.columns = [
        "_".join(col).strip() if col[1] else col[0]
        for col in aggregated_2.columns.values
    ]

    final_aggregated = pd.merge(aggregated_1, aggregated_2, on="Iteration", how="outer")
    final_aggregated = final_aggregated.fillna(0)
    final_aggregated.to_csv(output_file + ".csv", index=False)
    print(f"Aggregated data saved to {output_file}")



def aggregate_progress_by_time(folder_path, output_file, time_interval=1, decimals=3):
    csv_files = [
        os.path.join(folder_path, f)
        for f in os.listdir(folder_path)
        if f.endswith(".csv")
    ]

    if not csv_files:
        print("No CSV files found in the folder.")
        return

    overall_min_time = float("inf")
    overall_max_time = float("-inf")

    for file in csv_files:
        df = pd.read_csv(file)
        overall_min_time = min(overall_min_time, df["Time"].min())
        overall_max_time = max(overall_max_time, df["Time"].max())

    time_range = np.arange(
        overall_min_time, overall_max_time + time_interval, time_interval
    )

    interpolated_dataframes = []
    for file in csv_files:
        print(f"Loading {file}...")
        df = pd.read_csv(file)
        df.replace(-1, np.nan, inplace=True) 

        df = (
            df.set_index("Time")
            .reindex(time_range)
            .ffill()
            .reset_index()
        )
        df.rename(columns={"index": "Time"}, inplace=True)
        interpolated_dataframes.append(df)

    combined_data = pd.concat(interpolated_dataframes, ignore_index=True)
    aggregated_data = (
        combined_data.groupby("Time").agg({"Progress": ["mean", "std"]}).reset_index()
    )
    aggregated_data.columns = [
        "_".join(col).strip() if col[1] else col[0]
        for col in aggregated_data.columns.values
    ]

    aggregated_data.fillna(0, inplace=True)
    aggregated_data = aggregated_data.round(decimals)
    aggregated_data.to_csv(output_file + "_by_time.csv", index=False)
    print(f"Aggregated data across runs saved to {output_file}")


folder = "parameter_test/random/N"
name = "N10"
input_folder = folder + "/" + name
output_file = "final_data/N" + "/" + name

aggregate_by_iteration(input_folder, output_file)

aggregate_progress_by_time(input_folder, output_file, time_interval=1, decimals=3)
