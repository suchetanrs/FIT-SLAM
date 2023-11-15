import matplotlib.pyplot as plt
import csv

# Initialize elapsed time and counts
elapsed_time = []
unknown_counts = []
unknown_counts2 = []
elapsed_time2 = []
unknown_counts3 = []
elapsed_time3 = []
first_time_1 = 0
first_time_2 = 0
first_time_3 = 0
flag1 = False
flag2 = False
flag3 = False
# Define the CSV file name where data was saved
input_csv_file = '/home/agx/active_slam/active-slam-suchetan/ros2_ws/310900_greedy_frontier_map_data_coverage_.csv'
input_csv_file2 = '/home/agx/active_slam/active-slam-suchetan/ros2_ws/311251_random_frontier_map_data_coverage_.csv'
input_csv_file3 = '/home/agx/active_slam/active-slam-suchetan/ros2_ws/311624_ours_frontier_map_data_coverage_.csv'

def read_csv_data():
    global elapsed_time, unknown_counts, unknown_counts2, elapsed_time2, elapsed_time3, first_time_1, first_time_2, first_time_3, flag1, flag2
    with open(input_csv_file, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row if present
        for row in reader:
            current_time, unknown_count = map(float, row)
            if(flag1==False):
                first_time_1 = current_time
                flag1=True
            elapsed_time.append(current_time - first_time_1)
            unknown_counts.append(unknown_count)

    with open(input_csv_file2, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row if present
        for row in reader:
            current_time, unknown_count2 = map(float, row)
            if(flag2==False):
                first_time_2 = current_time
                flag2=True
            elapsed_time2.append(current_time - first_time_2)
            unknown_counts2.append(unknown_count2)

    with open(input_csv_file3, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row if present
        for row in reader:
            current_time, unknown_count3 = map(float, row)
            if(flag2==False):
                first_time_3 = current_time
                flag3=True
            elapsed_time3.append(current_time - first_time_3)
            unknown_counts3.append(unknown_count3)

def update_plot():
    global elapsed_time, unknown_counts, unknown_counts2, elapsed_time2, unknown_counts3, elapsed_time3
    # global elapsed_time, unknown_counts
    # Clear the current figure and plot the data
    plt.clf()
    #plt.plot(elapsed_time, unknown_counts, label='Number of unknown cells - random information frontier', color='red', linestyle='-')
    plt.xlabel('Elapsed Time (s)')
    plt.ylabel('Number of unknown cells')
    plt.title('Unknown Portion of the Map vs. Elapsed Time')
    #plt.ylim(0, 160000)
    # plt.xlim(0,1728)
    plt.legend()

    #plt.plot(elapsed_time2, unknown_counts2, label='Number of unknown cells - ours frontier', color='blue', linestyle='-')
    plt.plot(elapsed_time3, unknown_counts3, label='Number of unknown cells - greedy frontier', color='green', linestyle='-')
    plt.legend()

    # Pause to update the plot (you can adjust the pause duration)
    plt.grid()
    # plt.show()
    plt.pause(1)

def main():
    # global elapsed_time, unknown_counts, unknown_counts2, elapsed_time2
    # Read data from the CSV file
    read_csv_data()

    # Initialize the Matplotlib figure and start the plotting loop
    # plt.figure()
    # plt.ion()  # Turn on interactive mode
    # plt.show()

    # Update the plot using the provided function
    while(True):
        elapsed_time = []
        unknown_counts = []
        unknown_counts2 = []
        elapsed_time2 = []
        unknown_counts3 = []
        elapsed_time3 = []
        read_csv_data()
        update_plot()

    # Keep the plot window open
    # plt.show()

if __name__ == '__main__':
    main()
