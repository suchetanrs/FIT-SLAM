#include <iostream>
#include <functional>
#include <iomanip>
#include <limits>

// Function to generate a unique value using std::hash
double getUniqueValue(double a, double b) {
    std::hash<double> hash_fn;
    
    // Hash each double value
    std::size_t hash1 = hash_fn(a);
    std::size_t hash2 = hash_fn(b);

    // return hash1 ^ (hash2 << 1);
    // return hash1 ^ (hash2 << 1);
    return static_cast<double>(hash1 ^ (hash2 << 1));
}

int main() {
    double num2 = 3.14;
    double num1 = 2.71;

    auto uniqueValue = getUniqueValue(num1, num2);

    std::cout << "The unique value (hash) of " << num1 << " and " << num2 << " is: " << std::fixed << std::setprecision(15) << uniqueValue << std::endl;

    return 0;
}