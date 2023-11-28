import numpy as np
import matplotlib.pyplot as plt

def plot_coverage_vs_time(input_csv_files):
    # Create a list to store data from all files
    all_data = []

    # Read and process data from each CSV file
    for input_csv_file in input_csv_files:
        data = np.genfromtxt(input_csv_file, delimiter=',')
        time = data[:, 0]
        coverage = data[:, 1]
        max_coverage = data[0, 1]
        # percentage_coverage = (coverage / max_coverage) * 100
        percentage_coverage = (25000 - (max_coverage - coverage)) / 25000 * 100
        time -= time[0]
        mask = time < 3400
        time = time[mask]
        percentage_coverage = percentage_coverage[mask]
        all_data.append((time, percentage_coverage))

    # Create the plot for all files
    plt.figure(figsize=(10, 6))
    count = 0
    for time, percentage_coverage in all_data:
        print(type(time))
        np.savetxt(str(count) + ".csv", np.column_stack((time, percentage_coverage)), delimiter=',', header='Time,MapData', fmt='%.5f')
        count +=1
        plt.plot(time, percentage_coverage, marker='o', linestyle='-')

    plt.title('Percentage Map Covered vs. Time')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Percentage Map Covered (%)')
    plt.grid(True)
    plt.xlim((0,3400))
    plt.legend(input_csv_files)  # Add legends for each file
    plt.show()

# List of input CSV files
input_csv_files = [
    '/media/ssd-11/agx-xavier-backup/31exp/310900_greedy_frontier_map_data_coverage_.csv',
    '/media/ssd-11/agx-xavier-backup/31exp/311251_random_frontier_map_data_coverage_.csv',
    '/media/ssd-11/agx-xavier-backup/31exp/311740_ours_frontier_map_data_coverage_.csv'
]

# Call the function to plot data from all files
plot_coverage_vs_time(input_csv_files)
