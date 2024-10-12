import matplotlib.pyplot as plt
import csv
import time

plt.figure(figsize=(10, 6))
# Create the plot
# while(True):
# Define the paths to your two CSV files
csv_file_path_1 = '/root/compfslam2/playpen/occupancy_grid_count_20240923_201245.csv'  # Replace with the correct path to your first CSV file
csv_file_path_2 = '/root/compfslam2/playpen/occupancy_grid_count_20240923_201245.csv'  # Replace with the correct path to your second CSV file
csv_file_path_3 = '/root/compfslam2/playpen/occupancy_grid_count_20240923_201245.csv'  # Replace with the correct path to your second CSV file
csv_file_path_4 = '/root/compfslam2/playpen/occupancy_grid_count_20240923_203754.csv'

# Initialize lists to store the data for both files
times_1, cells_1 = [], []
times_2, cells_2 = [], []
times_3, cells_3 = [], []
times_4, cells_4 = [], []

# Function to read data from a CSV file
def read_csv_data(file_path, times, cells):
    with open(file_path, mode='r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            time = float(row[0])
            cell = int(row[1])
            times.append(time)
            cells.append(cell)

# Read the data from both CSV files
read_csv_data(csv_file_path_1, times_1, cells_1)
read_csv_data(csv_file_path_2, times_2, cells_2)
read_csv_data(csv_file_path_3, times_3, cells_3)
read_csv_data(csv_file_path_4, times_4, cells_4)

plt.title('Comparison of Explored Cells Over Time')
plt.xlabel('Time (seconds)')
plt.ylabel('Explored Cells')
plt.grid(True)
plt.plot(times_1, cells_1, marker='o', linestyle='-', label='GBPlanner2', markersize=3, color='blue')
plt.plot(times_2, cells_2, marker='o', linestyle='-', label='Our exploration algorithm', color='red', markersize=3)
plt.plot(times_3, cells_3, marker='o', linestyle='-', label='Our exploration algorithm', color='green', markersize=3)
plt.plot(times_4, cells_4, marker='o', linestyle='-', label='Our exploration algorithm', color='orange', markersize=3)
plt.tight_layout()
# plt.pause(1)
plt.legend()
plt.show()
time.sleep(60)