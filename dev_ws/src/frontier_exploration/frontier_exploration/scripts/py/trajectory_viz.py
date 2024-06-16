import matplotlib.pyplot as plt
import csv

# Initialize lists to store data
time = []
x_values = []
y_values = []

time2 = []
x_values2 = []
y_values2 = []

# Read data from CSV file
with open('/home/suchetan/2_loc_xy_2.csv', 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    for row in csv_reader:
        time.append(float(row[0]))
        x_values.append(float(row[1]))
        y_values.append(float(row[2]))

with open('/home/suchetan/2_gt_xy_2.csv', 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    for row in csv_reader:
        time2.append(float(row[0]))
        x_values2.append(float(row[1]))
        y_values2.append(float(row[2]))

# Create a figure and axis
plt.figure(figsize=(10, 6))
plt.title('Trajectory Visualization')
plt.xlabel('X Value')
plt.ylabel('Y Value')

# Plot the trajectory
plt.plot(x_values, y_values, label='Loc', marker='o', markersize=3, linestyle='-', color='b')
plt.plot(x_values2, y_values2, label='GT', marker='o', markersize=3, linestyle='-', color='r')
# Add labels to data points (optional)
# for i, txt in enumerate(time):
#     plt.annotate(f'Time: {txt:.2f}', (x_values[i], y_values[i]), textcoords='offset points', xytext=(0,10), ha='center')

# Add legend
plt.legend()

# Show the plot
plt.grid()
plt.show()