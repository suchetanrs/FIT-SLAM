import numpy as np

# Create a 5x5 matrix of frontier_distance with fixed values
frontier_distance = np.array([
    [0.1, 0.3, 0.2, 0.4, 0.5],
    [0.4, 0.1, 0.3, 0.5, 0.2],
    [0.2, 0.5, 0.1, 0.3, 0.4],
    [0.5, 0.2, 0.4, 0.1, 0.3],
    [0.3, 0.4, 0.5, 0.2, 0.1]
])

# Print the original frontier_distance matrix
print("Frontier Distance Matrix:")
print(frontier_distance)

# Perform the ordering of distances
order = np.argsort(frontier_distance, axis=1)
print("\nOrder Matrix:")
print(order)

# Perform the ranking based on the order
ranks = np.argsort(order, axis=1)
print("\nRanks Matrix:")
print(ranks)