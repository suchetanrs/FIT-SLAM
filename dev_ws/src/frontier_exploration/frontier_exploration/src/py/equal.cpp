#include <iostream>
#include <vector>

int main() {
    // Define two vectors of ints
    std::vector<int> vector1 = {1, 2, 3, 4, 5};
    std::vector<int> vector2 = {1, 2, 3, 5, 4}; // Same elements as vector1

    // Check if the vectors are equal
    bool areEqual = (vector1 == vector2);

    // Output the result
    if (areEqual) {
        std::cout << "The vectors are equal." << std::endl;
    } else {
        std::cout << "The vectors are not equal." << std::endl;
    }

    return 0;
}