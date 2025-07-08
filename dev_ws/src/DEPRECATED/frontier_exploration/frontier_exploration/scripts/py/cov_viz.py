import matplotlib.pyplot as plt
import csv
from datetime import datetime

# Read the CSV file and parse the data
timestamps = []
values = []
timestamps2 = []
values2 = []
timestamps3 = []
values3 = []

with open('/media/SSD_31/agx-xavier-backup/31exp/310900_greedy_loc_covariance_.csv', 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    for row in csv_reader:
        timestamp_str, value_str = row
        timestamp = float(timestamp_str)
        value = float(value_str)
        timestamps.append(timestamp)
        values.append(value)

with open('/media/SSD_31/agx-xavier-backup/31exp/310900_greedy_loc_covariance_.csv', 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    for row in csv_reader:
        timestamp_str, value_str = row
        timestamp2 = float(timestamp_str)
        value2 = float(value_str)
        timestamps2.append(timestamp2)
        values2.append(value2)

with open('/media/SSD_31/agx-xavier-backup/31exp/310900_greedy_loc_covariance_.csv', 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    for row in csv_reader:
        timestamp_str, value_str = row
        timestamp3 = float(timestamp_str)
        value3 = float(value_str)
        timestamps3.append(timestamp3)
        values3.append(value3)

# Calculate time elapsed since the first timestamp
start_time = timestamps[0]
time_elapsed = [(timestamp - start_time) for timestamp in timestamps]

start_time2 = timestamps2[0]
time_elapsed2 = [(timestamp2 - start_time2) for timestamp2 in timestamps2]

start_time3 = timestamps3[0]
time_elapsed3 = [(timestamp3 - start_time3) for timestamp3 in timestamps3]


# Create the plot
plt.figure(figsize=(10, 6))
plt.plot(time_elapsed, values, linestyle='-', label='Greedy frontier', linewidth=5, color='r')
#plt.plot(time_elapsed2, values2, linestyle='-', label='Random frontier', linewidth=5, color='b')
#plt.plot(time_elapsed3, values3, linestyle='-', label='Ours', linewidth=5, color='g')
plt.title('Covariance Trace vs Time Elapsed')
plt.xlabel('Time Elapsed (s)')
plt.ylabel('Covariance Trace (y)')
# plt.xlim(0, 2500)
plt.legend()
plt.grid(True)

# Show the plot
plt.show()
