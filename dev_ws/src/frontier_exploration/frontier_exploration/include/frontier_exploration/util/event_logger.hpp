#ifndef EVENT_LOGGER_HPP_
#define EVENT_LOGGER_HPP_

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

#include "frontier_exploration/util/logger.hpp"

class EventLogger {
public:
    static EventLogger& getInstance(const std::string &baseFilename = "defaultFilename")
    {
        std::lock_guard<std::mutex> lock(instanceMutex_);
        if(eventLoggerPtr_ == nullptr)
            eventLoggerPtr_.reset(new EventLogger(baseFilename));
        return *eventLoggerPtr_;
    }

    void startEvent(const std::string& key, int eventLevel);

    /**
     * eventLevel:
     * 0 - MODULE LEVEL
     * 1 - SUBMODULE_LEVEL
     * 2 - EVENT LEVEL
     */
    void endEvent(const std::string& key, int eventLevel);

    /**
     * Returns in seconds
     */
    double getTimeSinceStart(const std::string& key, int eventLevel);

private:
    // Delete copy constructor and assignment operator to prevent copying
    EventLogger(const EventLogger&) = delete;
    EventLogger& operator=(const EventLogger&) = delete;
    EventLogger(const std::string& baseFilename);

    static std::unique_ptr<EventLogger> eventLoggerPtr_;
    static std::mutex instanceMutex_;
    std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> startTimes;
    std::string csvFilename;
    int serialNumber;
    std::mutex mapMutex;
};

inline EventLogger& eventLoggerInstance = EventLogger::getInstance();

class Profiler
{
  public:
    Profiler(const std::string & functionName) : functionName(functionName)
    {
        eventLoggerInstance.startEvent(functionName + "_profiler", 2);
    }

    ~Profiler()
    {
        eventLoggerInstance.endEvent(functionName + "_profiler", 2);
    }

  private:
    std::string functionName;
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
};

#define PROFILE_FUNCTION Profiler profiler_instance(__func__);
#endif // COLOR_H