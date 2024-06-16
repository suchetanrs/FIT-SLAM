import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file into a DataFrame
df = pd.read_csv('/home/suchetan/3D_Occ_Info/3_opt_map_gt_3.csv', header=None)

# Extract the data for Pose 1
pose1_x = df.iloc[:, 1]  # Assuming position.x is in the second column (index 1)
pose1_y = df.iloc[:, 2]  # Assuming position.y is in the third column (index 2)

# Extract the data for Pose 2
pose2_x = df.iloc[:, 8]  # Assuming position.x for Pose 2 is in the fifth column (index 4)
pose2_y = df.iloc[:, 9]  # Assuming position.y for Pose 2 is in the sixth column (index 5)

# Create a scatter plot
plt.figure(figsize=(8, 6))
plt.plot(pose1_x, pose1_y, label='SLAM', marker='o', linestyle='-', color='b')
plt.plot(pose2_x, pose2_y, label='GT', marker='x', linestyle='-', color='r')

plt.xlabel('Position X')
plt.ylabel('Position Y')
plt.title('SLAM vs GT Trajectory comparisions')
plt.legend()
plt.grid(True)

# Show the plot
plt.show()
