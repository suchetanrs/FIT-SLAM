#include <iostream>
#include <map>
#include <vector>
#include <mutex>

template <typename T>
size_t getSizeOfVector(const std::vector<T>& vec) {
    return sizeof(vec) + vec.capacity() * sizeof(T);
}

template <typename Key, typename Value>
size_t getSizeOfMap(const std::map<Key, Value>& map) {
    size_t size = sizeof(map);
    for (const auto& pair : map) {
        size += sizeof(pair);                 // size of the pair itself
        size += sizeof(pair.first);           // size of the key
        size += getSizeOfVector(pair.second); // size of the vector (value)
    }
    return size;
}

int main() {
    // Your map type, e.g., std::map<int, std::vector<int>>
    std::map<int, std::vector<int>> spatial_hash_map_;

    // Fill the map with some data for demonstration
    for (int i = 0; i < 25; ++i) {
        std::vector<int> new_frontier = {1, 2, 3, 4, 5};
        spatial_hash_map_[i] = new_frontier;
    }

    // Calculate the size in bytes
    size_t size_in_bytes = getSizeOfMap(spatial_hash_map_);

    // Convert bytes to megabytes (1 MB = 1,048,576 bytes)
    double size_in_megabytes = static_cast<double>(size_in_bytes) / (1024 * 1024);

    // Print the size in MB
    std::cout << "Size of spatial_hash_map_ in MB: " << size_in_megabytes << " MB" << std::endl;

    return 0;
}