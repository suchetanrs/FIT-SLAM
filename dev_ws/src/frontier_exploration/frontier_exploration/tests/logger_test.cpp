#include "frontier_exploration/util/logger.hpp"
#include "frontier_exploration/Frontier.hpp"

int main() {
    Frontier frontier;
    frontier.setGoalPoint(9.0, 98.0);
    frontier.setPathLength(5.0);
    frontier.setPathLengthInM(66);
    // Test the logging macros
    LOG_TRACE("This is a TRACE message." << 5);
    LOG_DEBUG("This is a DEBUG message.\n" << frontier);
    LOG_INFO("This is an INFO message.");
    LOG_WARN("This is a WARN message.");
    LOG_ERROR("This is an ERROR message.");
    LOG_CRITICAL("This is a CRITICAL message.");
    LOG_FATAL("This is a FATAL message.");
    LOG_HIGHLIGHT("This is a FATAL message.");

    return 0;
}