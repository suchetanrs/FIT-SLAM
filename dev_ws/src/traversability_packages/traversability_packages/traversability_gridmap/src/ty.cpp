#include <iostream>
#include <map>

int main() {
    std::map<int, int> myMap;

    // Using [] operator
    myMap[1] = 0;  // Inserts {1, true} into the map
    myMap[2] = 1; // Inserts {2, false} into the map

    std::cout << "Using [] operator:" << std::endl;
    std::cout << "Key 1: " << myMap[1] << std::endl; // Accessing existing key
    std::cout << "Key 2: " << myMap[2] << std::endl; // Accessing existing key
    std::cout << "Key 3: " << myMap[3] << std::endl; // Accessing non-existing key, inserts {3, false}

    std::cout << std::endl;

    // Using at() function
    std::cout << "Using at() function:" << std::endl;
    std::cout << "Key 1: " << myMap.at(1) << std::endl; // Accessing existing key
    std::cout << "Key 2: " << myMap.at(2) << std::endl; // Accessing existing key
    try {
        std::cout << "Key 3: " << myMap.at(3) << std::endl; // Accessing non-existing key, throws std::out_of_range
    } catch (const std::out_of_range& e) {
        std::cerr << "Caught exception: " << e.what() << std::endl;
    }

    return 0;
}