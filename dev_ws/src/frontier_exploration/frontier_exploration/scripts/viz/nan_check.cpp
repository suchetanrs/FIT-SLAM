#include <iostream>
#include <cmath>       // for std::isnan
#include <limits>      // for std::numeric_limits

int main() {
    // Assign NaN to a variable
    double value = std::numeric_limits<double>::quiet_NaN();
    
    // Check if the assigned value is NaN
    if (std::isnan(value)) {
        std::cout << "The value is NaN." << std::endl;
        std::cout << "Val" << value * 5.0 << std::endl;
    } else {
        std::cout << "The value is not NaN." << std::endl;
    }

    return 0;
}
