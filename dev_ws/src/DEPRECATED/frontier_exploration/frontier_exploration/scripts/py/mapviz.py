import matplotlib.pyplot as plt
import csv
import numpy as np

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
input_csv_file = '/root/refactored_old/scout_1_12904_ours_frontier_map_data_coverage.csv'
input_csv_file2 = '/root/scout_1_12904_ours_frontier_map_data_coverage.csv'
input_csv_file3 = '/root/minPos2/scout_1_12904_ours_frontier_map_data_coverage.csv'

def read_csv_data():
    global elapsed_time, unknown_counts, unknown_counts2, elapsed_time2, elapsed_time3, first_time_1, first_time_2, first_time_3, flag1, flag2, flag3
    count = 0
    with open(input_csv_file, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row if present
        for row in reader:
            count += 1
            current_time, unknown_count = map(float, row)
            if(flag1==False):
                first_time_1 = current_time
                flag1=True
            elapsed_time.append(current_time - first_time_1)
            # elapsed_time.append(count)
            unknown_counts.append(unknown_count)

    count = 0
    with open(input_csv_file2, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row if present
        for row in reader:
            count += 1
            current_time, unknown_count2 = map(float, row)
            if(flag2==False):
                first_time_2 = current_time
                flag2=True
            elapsed_time2.append(current_time - first_time_2)
            # elapsed_time2.append(count)
            unknown_counts2.append(unknown_count2)

    count = 0
    with open(input_csv_file3, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row if present
        for row in reader:
            count += 1
            current_time, unknown_count3 = map(float, row)
            if(flag3==False):
                first_time_3 = current_time
                flag3=True
            elapsed_time3.append(current_time - first_time_3)
            # elapsed_time3.append(count)
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

    plt.plot(elapsed_time, unknown_counts, label='Number of unknown cells', color='green', linestyle='-')
    plt.plot(elapsed_time2, unknown_counts2, label='Number of unknown cells hungarian', color='blue', linestyle='-')
    # plt.plot(elapsed_time3, unknown_counts3, label='Number of unknown cells minPos - scout-2', color='red', linestyle='-')
    # plt.plot(elapsed_time, unknown_counts, label='Number of unknown cells hungarian - scout-1', color='green', linestyle='-')
    plt.legend()

    # Pause to update the plot (you can adjust the pause duration)
    plt.grid()
    # plt.show()
    plt.pause(1)

def update_plot_fit():
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

    # Fit a linear model to the data
    coefficients = np.polyfit(elapsed_time, unknown_counts, 10)
    coefficients2 = np.polyfit(elapsed_time2, unknown_counts2, 10)
    coefficients3 = np.polyfit(elapsed_time3, unknown_counts3, 10)

    # Create a polynomial object with the coefficients
    polynomial = np.poly1d(coefficients)
    polynomial2 = np.poly1d(coefficients2)
    polynomial3 = np.poly1d(coefficients3)

    # Generate values for plotting the fit line
    fit_line = polynomial(elapsed_time)
    fit_line2 = polynomial2(elapsed_time2)
    fit_line3 = polynomial3(elapsed_time3)

    # Plot the data
    # plt.plot(elapsed_time, unknown_counts, 'o', label='Number of unknown cells', color='green')
    # plt.plot(elapsed_time2, unknown_counts2, 'o', label='Number of unknown cells hungarian - scout-1', color='blue')
    # plt.plot(elapsed_time3, unknown_counts3, 'o', label='Number of unknown cells minPos - scout-2', color='red')

    # Plot the fitted lines
    plt.plot(elapsed_time, fit_line, label='Exploration rate - old multirobot', color='green', linestyle='-')
    plt.plot(elapsed_time2, fit_line2, label='Exploration rate - new multirobot', color='blue', linestyle='-')
    # plt.plot(elapsed_time3, fit_line3, label='Fitted line minPos - scout-2', color='red', linestyle='-')

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
    # while(True):
    #     elapsed_time = []
    #     unknown_counts = []
    #     unknown_counts2 = []
    #     elapsed_time2 = []
    #     unknown_counts3 = []
    #     elapsed_time3 = []
    #     read_csv_data()
    #     update_plot()
    update_plot_fit()
    # update_plot()

    # Keep the plot window open
    plt.show()

if __name__ == '__main__':
    main()
