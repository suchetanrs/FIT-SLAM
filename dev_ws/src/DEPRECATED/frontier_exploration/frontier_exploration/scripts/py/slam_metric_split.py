import csv

# Define the input CSV file name and the output file names for ground truth and second trajectory
input_csv_file = '/home/suchetan/ours_opt_map_gt_3.csv'
ground_truth_output_file = 'ground_truth_ours.txt'
second_trajectory_output_file = 'slam_trajectory_ours.txt'

# Initialize an index counter
index = 1.0
headers = "timestamp tx ty tz qx qy qz qw"
# Open the input CSV file for reading
with open(input_csv_file, 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    
    # Open the output files for writing
    with open(ground_truth_output_file, 'w') as ground_truth_file, open(second_trajectory_output_file, 'w') as second_trajectory_file:
        ground_truth_file.write(headers + '\n')
        second_trajectory_file.write(headers + '\n')
        for row in csv_reader:
            # Split the row into two parts, each with 7 columns
            ground_truth_data = f'{index} ' + ' '.join(row[7:]) + '\n'
            second_trajectory_data = f'{index} ' + ' '.join(row[:7]) + '\n'
            
            # Write the data to the respective output files
            ground_truth_file.write(ground_truth_data)
            second_trajectory_file.write(second_trajectory_data)
            
            # Increment the index
            index += 1

print("Separation complete. Two text files created: ground_truth.txt and second_trajectory.txt")