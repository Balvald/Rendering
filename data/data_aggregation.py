"""aggregation script for fouranimals dataset."""

import os
import matplotlib.pyplot as plt
import re

# all files in the same directory as this file are named in the form "log-fouranimals.obj-A-B-C-D.txt" where A,B,C and D are integers for method, max_depth, max_trig and bins_count.


def parse_filename(filename) -> tuple:
    """Parse the filename and return method, max_depth, max_trig, bins_count as integers, or None if invalid."""
    if filename.startswith('log-fouranimals.obj-') and filename.endswith('.txt'):
        parts = filename[len('log-fouranimals.obj-'):-4].split('-')
        print(f"Parsing filename: {filename}, parts: {parts}")
        if len(parts) == 4:
            try:
                return tuple(map(int, parts))
            except ValueError:
                return None
    return None


def get_axis_value(axis, method, max_depth, max_trig, bins_count) -> str:
    """Return the value for the specified axis."""
    axis_map = {
        'method': method,
        'max_depth': max_depth,
        'max_trig': max_trig,
        'bins_count': bins_count
    }
    return axis_map.get(axis)


def aggregate_data(axis, directory='.') -> dict:
    """
    Aggregate data from files in the specified directory along the given axis.

    Parameters:
    axis (str): The axis to aggregate by ('method', 'max_depth', 'max_trig', 'bins_count').
    directory (str): The directory containing the data files.

    Returns:
    dict: A dictionary with aggregated results for each unique value of the specified axis.
    """
    aggregated_results = {}

    for filename in os.listdir(directory):
        if not filename.endswith('.txt'):
            continue
        print(f"Processing file: {filename}")
        parsed = parse_filename(filename)
        if parsed is None:
            continue
        method, max_depth, max_trig, bins_count = parsed
        key = get_axis_value(axis, method, max_depth, max_trig, bins_count)
        if key not in aggregated_results:
            aggregated_results[key] = []
        # Here you would read the file and extract relevant data to aggregate
        # For simplicity, we will just append the filename as a placeholder
        aggregated_results[key].append(filename)

    return aggregated_results


if __name__ == "__main__":
    axis = 'method'  # Change this to 'max_depth', 'max_trig', or 'bins_count' as needed
    results = aggregate_data(axis, directory='.')

    regex = re.compile(r"([\d.e\+-]*) seconds.")

    # for each file we now have to extract the data and aggregate it
    aggregated_data = {}
    files = list(results.values())[0]
    for filename in files:
        print(f"Aggregating data from file: {filename}")
        with open(filename, 'r') as f:
            content = f.read()
            matches = regex.findall(content)
            for match in matches:
                if match:
                    time = float(match)
                    if time not in aggregated_data:
                        aggregated_data[time] = []
                    aggregated_data[time].append(filename)
    # Plotting the aggregated data
    plt.figure(figsize=(10, 6))
    for time, files in aggregated_data.items():
        plt.plot(files, [time] * len(files), marker='o', linestyle='', label=f'Time: {time} seconds')
    plt.xlabel('Files')
    plt.ylabel('Time (seconds)')
    plt.title(f'Aggregated Data by {axis}')
    plt.legend()
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.show()
