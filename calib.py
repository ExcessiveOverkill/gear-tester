import numpy as np
import matplotlib.pyplot as plt

def generatePolynomial(filename: str, min_vel: float, max_vel: float) -> np.poly1d:
    data = np.load(filename)
    # Filter the data to only include points within the specified velocity range
    data = data[(data[:, 0] >= min_vel) & (data[:, 0] <= max_vel)]

    # Assuming the data is a 2D array where the first column is speed (x) and the second is friction (y)
    x = data[:, 0]  # Speed values
    y = data[:, 1]  # Friction values

    # Fit a polynomial to the data
    degree = 3  # Degree of the polynomial (you can change this value)
    coefficients = np.polyfit(x, y, degree)
    return np.poly1d(coefficients)


if __name__ == '__main__':

    # Load data from the .npy file
    filename = 'negative_calibration_raw_data.npy'
    MIN = 5
    MAX = 200
    polynomial = generatePolynomial(filename, -MAX, -MIN)

    data = np.load(filename)
    # Filter the data to only include points within the specified velocity range
    data = data[(data[:, 0] >= -MAX) & (data[:, 0] <= -MIN)]

    x = data[:, 0]  # Speed values
    y = data[:, 1]  # Friction values

    # Generate x values for plotting the polynomial fit
    x_fit = np.linspace(min(x), max(x), 500)
    y_fit = polynomial(x_fit)

    # Plot the original data points
    plt.scatter(x, y, color='blue', label='Data Points')

    # Plot the polynomial fit line
    plt.plot(x_fit, y_fit, color='red', label=f'Polynomial Fit')

    # Add labels and title
    plt.xlabel('Speed')
    plt.ylabel('Friction')
    plt.title('Speed vs. Friction')

    # Add a legend
    plt.legend()

    # Display the plot
    plt.show()
