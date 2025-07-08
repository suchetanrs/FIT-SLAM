import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi, voronoi_plot_2d

# Define the three points
points = np.array([[0, 0], [1, 1], [0.5, 1.5]])

# Compute the Voronoi diagram
vor = Voronoi(points)

# Plot the Voronoi diagram
fig, ax = plt.subplots()
voronoi_plot_2d(vor, ax=ax, show_vertices=False, line_colors='orange', line_width=2, line_alpha=0.6, point_size=15)

# Plot the points
ax.plot(points[:, 0], points[:, 1], 'bo')

# Set plot limits
ax.set_xlim(-1, 2)
ax.set_ylim(-1, 2)

# Display the plot
plt.show()