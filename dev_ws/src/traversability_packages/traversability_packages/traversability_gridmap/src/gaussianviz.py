import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

N = 0
sx = 0
sy = 0
sx2 = 0
sy2 = 0
sxy = 0

def assignsxsy(p3d):
    global N, sx, sy, sx2, sy2, sxy
    N +=1
    sx += p3d[0]
    sy += p3d[1]
    
    sx2 += p3d[0] * p3d[0]
    sy2 += p3d[1] * p3d[1]
    
    sxy += p3d[0] * p3d[1]

def get(coordinates):
    mean_x = sx/N
    mean_y = sy/N
    print(f"Means:  {mean_x} , {mean_y}")



    # Calculate the covariance matrix
    centered_coords = coordinates - np.array([mean_x, mean_y])
    covariance_matrix = np.dot(centered_coords.T, centered_coords) / len(coordinates)

    print(f"The covariance matrix method 1 is: \n  {covariance_matrix}")

    covariance_matrix = np.zeros((2, 2))
    covariance_matrix[0,0] = (sx2 + (N*mean_x*mean_x) - (2*mean_x*sx))/N
    covariance_matrix[0,1] = (sxy - (mean_y*sx) - (mean_x*sy) + (N*mean_x*mean_y))/N
    covariance_matrix[1,0] = (sxy - (mean_y*sx) - (mean_x*sy) + (N*mean_x*mean_y))/N
    covariance_matrix[1,1] = (sy2 + (N*mean_y*mean_y) - (2*mean_y*sy))/N


    print(f"The covariance matrix method 2 is: \n  {covariance_matrix}")

    mu = np.array([mean_x, mean_y])
    return mu, covariance_matrix

def plot_gaussian_ellipse(mean, cov, n_std=2, ax=None, **kwargs):
    """
    Plot an ellipse representing a Gaussian distribution.

    Parameters:
        mean (array-like): Mean of the Gaussian distribution.
        cov (array-like): Covariance matrix of the Gaussian distribution.
        n_std (float): Number of standard deviations to determine ellipse size.
        ax (matplotlib.axes._axes.Axes, optional): Axes on which to plot the ellipse.
        **kwargs: Additional arguments to pass to the Ellipse constructor.

    Returns:
        matplotlib.patches.Ellipse: The plotted ellipse.
    """
    if ax is None:
        ax = plt.gca()

    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    order = eigenvalues.argsort()[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]

    angle = np.degrees(np.arctan2(*eigenvectors[:, 0][::-1]))
    width, height = 2 * n_std * np.sqrt(eigenvalues)

    ellipse = Ellipse(xy=mean, width=width, height=height, angle=angle, **kwargs)
    ax.add_patch(ellipse)
    return ellipse

data = np.array([[1, 2], [2, 3], [3, 4], [4, 5], [5, 6], [6,5], [7,4] , [8,3] , [9,2]])
data = np.random.rand(100, 2)
# Call assignsxsy for each element in data
for p3d in data:
    assignsxsy(p3d)


mean, covariance = get(data)
plt.scatter(data[:, 0], data[:, 1], marker='.', label='Data Points')
# Plot the Gaussian ellipse
plot_gaussian_ellipse(mean, covariance, n_std=2, ax=plt.gca(), color='red', alpha=0.3, label='2 Std Dev')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Gaussian Distribution with Ellipse')
plt.legend()
plt.axis('equal')
plt.show()