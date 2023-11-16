#include <iostream>
#include <Eigen/Dense>

int main() {
    // Create a 3D vector
    Eigen::Vector3d vector(1.0, 2.0, 3.0);

    // Subtract a constant value from the vector
    double constantValue = 0.5;
    vector -= constantValue;
    vector.x();

    // Print the result
    std::cout << "Result: " << vector << std::endl;

    return 0;
}