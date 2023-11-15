import matplotlib.pyplot as plt

# Read coordinatesmap from the text file
coordinatesmap = []
coordinatesworld = []
coordinatestraced = []
with open("/home/suchetan/coordinatesmap.txt", "r") as file:
    for line in file:
        x, y = map(float, line.strip().split())
        coordinatesmap.append((x, y))

with open("/home/suchetan/coordinatesworld.txt", "r") as file:
    for line in file:
        x, y = map(float, line.strip().split())
        coordinatesworld.append((x, y))

with open("/home/suchetan/coordinatestraced.txt", "r") as file:
    for line in file:
        x, y = map(float, line.strip().split())
        coordinatestraced.append((x, y))

# Extract x and y values from the coordinatesmap
x_values, y_values = zip(*coordinatesmap)
# Extract x and y values from the coordinatesmap
x_valuesw, y_valuesw = zip(*coordinatesworld)
# Extract x and y values from the coordinatesmap
x_valuest, y_valuest = zip(*coordinatestraced)

# Create a scatter plot using Matplotlib
plt.scatter(x_values, y_values, label="Coordinatesmap", color="blue", marker="o")
plt.scatter(x_valuesw, y_valuesw, label="Coordinatesworld", color="red", marker="o")
plt.scatter(x_valuest, y_valuest, label="Coordinatestraced", color="green", marker="x")
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Coordinate Visualization")
plt.legend()
plt.grid(True)

# Display the plot
plt.show()
