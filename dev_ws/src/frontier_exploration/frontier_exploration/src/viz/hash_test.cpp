#include <iostream>
#include <unordered_map>
#include <string>
#include <functional>
#include <limits>

class MyClass {
public:
    double double1;
    double double2;
    std::string str;
    float flt;
    bool bl;

    MyClass(double d1, double d2, const std::string& s, float f, bool b)
        : double1(d1), double2(d2), str(s), flt(f), bl(b) {}
};

// Custom hash function
struct MyClassHash {
    std::size_t operator()(const MyClass& mc) const {
        std::size_t h1 = std::hash<double>()(mc.double1);
        std::size_t h2 = std::hash<double>()(mc.double2);
        std::size_t h3 = std::hash<std::string>()(mc.str);
        // Combine the hash values using a technique similar to boost::hash_combine
        // return h1 ^ (h2 << 1) ^ (h3 << 2);
        return h1 ^ (h2 << 1);
    }
};

// Custom equality function
struct MyClassEqual {
    bool operator()(const MyClass& lhs, const MyClass& rhs) const {
        return lhs.double1 == rhs.double1 &&
               lhs.double2 == rhs.double2 &&
               lhs.str == rhs.str;
    }
};

int main() {
    // Create an unordered_map with MyClass as the key
    std::unordered_map<MyClass, std::string, MyClassHash, MyClassEqual> myMap;

    // Create some MyClass objects
    MyClass obj1(1.0, 2.0, "Hello", 3.0f, true);
    MyClass obj2(4.0, 5.0, "World", 6.0f, false);
    MyClass obj3(4.0, 5.0, "World", 6.3f, true);
    MyClass obj4(4.0, 5.1, "World", 6.3f, true);

    // Add objects to the map
    myMap[obj1] = "Object 1";
    myMap[obj2] = "Object 2";
    myMap[obj3] = "Object 3";
    myMap[obj4] = "Object 4";

    // Access the objects in the map
    std::cout << "obj1: " << myMap.count(obj1) << std::endl;
    std::cout << "obj2: " << myMap.count(obj2) << std::endl;
    std::cout << "obj3: " << myMap.count(obj3) << std::endl;
    std::cout << "obj4: " << myMap.count(obj4) << std::endl;
    bool val = 5 > std::numeric_limits<double>::infinity();
    std::cout << val << std::endl;

    return 0;
}
