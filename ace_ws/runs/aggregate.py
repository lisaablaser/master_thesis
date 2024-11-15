import pandas as pd
import numpy as np
import os


def aggregate_runs(folder_path, output_file):
    # Get a list of all .csv files in the folder
    csv_files = [
        os.path.join(folder_path, f)
        for f in os.listdir(folder_path)
        if f.endswith(".csv")
    ]

    if not csv_files:
        print("No CSV files found in the folder.")
        return

    # Read all files and store them in a list
    dataframes = []
    for file in csv_files:
        print(f"Loading {file}...")
        df = pd.read_csv(file)
        dataframes.append(df)

    # Concatenate all dataframes along the rows (axis=0)
    combined_data = pd.concat(dataframes, ignore_index=True)

    # Replace invalid values (-1) with NaN
    combined_data.replace(-1, np.nan, inplace=True)

    # Aggregate the data by 'Iteration' and compute mean and std across runs
    aggregated_data = (
        combined_data.groupby("Iteration")
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
                "Progress": ["mean", "std"],
                "Planner": ["mean", "std"],
                "TrajectoryLength": ["mean", "std"],
                "MoveTime": ["mean", "std"],
                "MapUpdateTime": ["mean", "std"],
                "Gain": ["mean", "std"],
            }
        )
        .reset_index()
    )

    # Rename the multi-index columns
    aggregated_data.columns = [
        "_".join(col).strip() if col[1] else col[0]
        for col in aggregated_data.columns.values
    ]

    aggregated_data.fillna(0, inplace=True)

    # Save the aggregated data to a single output file
    aggregated_data.to_csv(output_file + ".csv", index=False)
    print(f"Aggregated data across runs saved to {output_file}")


def aggregate_progress_vs_time(folder_path, output_file, time_interval=10, decimals=3):
    # Get a list of all .csv files in the folder
    csv_files = [
        os.path.join(folder_path, f)
        for f in os.listdir(folder_path)
        if f.endswith(".csv")
    ]

    if not csv_files:
        print("No CSV files found in the folder.")
        return

    # Define the common time grid
    time_range = None

    interpolated_dataframes = []
    for file in csv_files:
        print(f"Loading {file}...")
        df = pd.read_csv(file)
        df.replace(-1, np.nan, inplace=True)  # Replace invalid values

        # Define the time grid based on the first file's range
        if time_range is None:
            time_range = np.arange(
                df["Time"].min(), df["Time"].max() + time_interval, time_interval
            )

        # Interpolate to the common time grid
        df = (
            df.set_index("Time")
            .reindex(time_range)
            .interpolate(method="linear")
            .reset_index()
        )
        df.rename(columns={"index": "Time"}, inplace=True)

        interpolated_dataframes.append(df)

    # Concatenate all dataframes along the rows (axis=0)
    combined_data = pd.concat(interpolated_dataframes, ignore_index=True)

    # Group by Time and calculate mean and std for Progress
    aggregated_data = (
        combined_data.groupby("Time").agg({"Progress": ["mean", "std"]}).reset_index()
    )

    # Rename the multi-index columns
    aggregated_data.columns = [
        "_".join(col).strip() if col[1] else col[0]
        for col in aggregated_data.columns.values
    ]

    # Replace any remaining NaN or empty values with 0
    aggregated_data.fillna(0, inplace=True)

    # Round numerical values to the specified number of decimal places
    aggregated_data = aggregated_data.round(decimals)

    # Save the aggregated data to a single output file
    aggregated_data.to_csv("aggregates/" + output_file + "_by_time.csv", index=False)
    print(f"Aggregated data across runs saved to {output_file}")


# Input folder containing the CSV files
name = "random_local"
input_folder = name
output_file = name

# Process all CSV files in the folder and aggregate them
aggregate_runs(input_folder, output_file)

aggregate_progress_vs_time(input_folder, output_file, time_interval=10, decimals=3)
