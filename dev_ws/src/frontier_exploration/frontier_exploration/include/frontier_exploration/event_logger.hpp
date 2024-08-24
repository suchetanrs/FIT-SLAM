#include <iostream>
#include <unordered_map>
#include <chrono>
#include <string>
#include <fstream>
#include <thread>
#include <mutex>
#include <random>
#include <ctime>

#include <iomanip>
#include <sstream>
#include <ctime>

class EventLogger {
public:
    EventLogger(const std::string& baseFilename);

    void startEvent(const std::string& key);

    void endEvent(const std::string& key);

private:
    std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> startTimes;
    std::string csvFilename;
    int serialNumber;
    std::mutex mapMutex;
};