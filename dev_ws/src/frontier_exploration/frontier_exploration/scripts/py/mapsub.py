import rclpy
from nav_msgs.msg import OccupancyGrid
import time
import matplotlib.pyplot as plt
import csv

# Initialize elapsed time and counts
# elapsed_time = []
# unknown_counts = []
# known_counts = []

# Get the program start time
start_time = time.time()

output_csv_file = 'map_data_info.csv'

def map_callback(msg):
    # This function will be called when a new message is received on the /map topic
    # You can process the occupancy grid data here
    # For example, you can access the data using msg.data

    # Extract map width and height
    map_width = msg.info.width
    map_height = msg.info.height
    print("Map Width:", msg.info.width)
    print("Map Height:", msg.info.height)

    # Iterate through the map data and print occupancy values
    unknown_count = 400 * 400
    starting_row = int(map_height / 2) - 200
    starting_col = int(map_width / 2) - 200
    for row in range(starting_row, starting_row + 400):
        for col in range(starting_col, starting_col + 400):
            # Calculate the index of the grid cell
            index = col + row * map_width
            try:
                occupancy_value = msg.data[index]
                if(occupancy_value != -1):
                    unknown_count -= 1
                    # print(f"Grid({col}, {row}) Occupancy Value: {occupancy_value}")
            except:
                print("CLIMB!" + str(time.time()))

    # Calculate elapsed time since program start
    current_time = time.time() - start_time
    # elapsed_time.append(current_time)
    # unknown_counts.append(unknown_count)

    # Update the plot
    # update_plot()
    write_to_csv(current_time, unknown_count)



def write_to_csv(current_time, unknown_count):
    with open(output_csv_file, mode='a', newline='') as file:
        writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        writer.writerow([current_time, unknown_count])


def main():
    rclpy.init()

    # Create a ROS 2 node
    node = rclpy.create_node('map_subscriber_node')

    # Create a subscription to the /map topic
    subscription = node.create_subscription(
        OccupancyGrid,
        '/map',
        map_callback,
        10  # Queue size
    )

    # Print a message indicating the subscription is active
    node.get_logger().info('Subscribed to /map topic')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Clean up and shutdown the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()