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
        elif len(parts) == 3:
            try:
                parts.append(-1)
                return tuple(map(int, parts))  # Assuming -1 for bins_count if not present
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
    axis = 'bins_count'  # Change this to 'method' 'max_depth', 'max_trig', or 'bins_count' as needed
    results = aggregate_data(axis, directory='.')

    print(f"Aggregated results for axis '{axis}': {results}")

    regex = re.compile(r"([\d.e\+-]*) seconds.")

    defined_method = 3
    defined_max_depth = 10
    defined_max_trig = 1024
    defined_bins_count = -1

    # Filter results based on defined variables (except for the free axis)
    filtered_results = {}
    for key, files in results.items():
        filtered_files = []
        for filename in files:
            parsed = parse_filename(filename)
            if parsed is None:
                continue
            method, max_depth, max_trig, bins_count = parsed

            # Check if file matches the defined constraints (skip the axis that is free)
            match = True
            if axis != 'method' and defined_method != -1 and method != defined_method:
                match = False
            if axis != 'max_depth' and defined_max_depth != -1 and max_depth != defined_max_depth:
                match = False
            if axis != 'max_trig' and defined_max_trig != -1 and max_trig != defined_max_trig:
                match = False
            if axis != 'bins_count' and defined_bins_count != -1 and bins_count != defined_bins_count:
                match = False

            if match:
                filtered_files.append(filename)

        if filtered_files:  # Only keep keys that have matching files
            filtered_results[key] = filtered_files

    # for each file we now have to extract the data and aggregate it
    aggregated_data = {}
    for key, files in filtered_results.items():
        for filename in files:
            print(f"Aggregating data from file: {filename}")
            with open(filename, 'r') as f:
                content = f.read()
                matches = regex.findall(content)
                for match in matches:
                    if match:
                        time = float(match)
                        if filename not in aggregated_data:
                            aggregated_data[filename] = []
                        aggregated_data[filename].append(time)

    method_used = ["Nothing", "BVH Splitting X", "Longest Extension", "SAH"]

    # Plotting the aggregated data
    plt.figure(figsize=(10, 6))
    print(aggregated_data)

    # sort aggregated_data by filenames
    aggregated_data = {k: aggregated_data[k] for k in sorted(aggregated_data, key=lambda x: (len(x), x))}

    depthvalues = []
    alltimes = []
    alltimes2 = []
    alltimes3 = []
    all_max_trig = []
    all_bins_count = []
    all_methods = []

    for key, times in aggregated_data.items():
        # filter for the axis in key (which is a filename)
        split = key.split('-')
        method_value = split[2]
        max_depth_value = split[3]
        max_trig_value = split[4].strip('.txt')
        if len(split) < 6:
            bins_count_value = -1
        else:
            bins_count_value = split[5].strip('.txt')

        # if int(max_depth_value) < 1:
        #     continue

        print(f"method: {method_value}, max_depth: {max_depth_value}, max_trig: {max_trig_value}, bins_count: {bins_count_value}")

        # build the label for the plot
        string = method_used[int(method_value)] + f", d: {max_depth_value}, m: {max_trig_value}, b: {bins_count_value}"

        # plt.plot(string, times[0], marker='o', linestyle='', label=f'{axis}: {key}')

        depthvalues.append(int(max_depth_value))
        alltimes.append(times[0])
        alltimes2.append(times[1])
        alltimes3.append(times[2])
        all_max_trig.append(int(max_trig_value))
        all_bins_count.append(int(bins_count_value))
        all_methods.append(method_used[int(method_value)])

        # plt.plot(key, times[1], marker='o', linestyle='', label=f'{axis}: {key}')
        # plt.plot(key, times[2], marker='o', linestyle='', label=f'{axis}: {key}')

    # plt.plot(all_max_trig, alltimes, marker='o', linestyle='-', label=f'{axis}: {key}')
    # plt.plot(all_max_trig, alltimes2, marker='o', linestyle='-', label=f'{axis}: {key}')
    # plt.plot(all_max_trig, alltimes3, marker='o', linestyle='-', label=f'{axis}: {key}')
    plt.plot(all_bins_count, alltimes, marker='o', linestyle='-', label=f'{axis}: {key}')
    # plt.plot(all_bins_count, alltimes2, marker='o', linestyle='-', label=f'{axis}: {key}')
    plt.plot(all_bins_count, alltimes3, marker='o', linestyle='-', label=f'{axis}: {key}')
    # plt.plot(depthvalues, alltimes, marker='o', linestyle='-', label=f'{axis}: {key}')
    # plt.plot(depthvalues, alltimes2, marker='o', linestyle='-', label=f'{axis}: {key}')
    # plt.plot(depthvalues, alltimes3, marker='o', linestyle='-', label=f'{axis}: {key}')
    # plt.plot(all_methods, alltimes, marker='o', linestyle='-', label=f'{axis}: {key}')
    # plt.plot(all_methods, alltimes2, marker='o', linestyle='-', label=f'{axis}: {key}')
    # plt.plot(all_methods, alltimes3, marker='o', linestyle='-', label=f'{axis}: {key}')


    plt.xlabel('Maximum Triangles per Leaf Node')
    plt.ylabel('Time (in seconds)')
    plt.title('Performance in relation to Max Triangles per leaf using Single Axis Splitting')
    # plt.legend()
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.show()
