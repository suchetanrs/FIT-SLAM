#ifndef COLOR_HPP_
#define COLOR_HPP_

#include <iostream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <frontier_msgs/msg/frontier.hpp>

// Color_id	    Color
//        1	    Aqua
//        2	    Green
//        3	    Blue
//        4	    Red
//        5	    Purple
//        6	    Yellow
//        7	    White
//        8	    Gray
//        A	    Light Green
//        0	    Black
//        9	    Light  Blue
//        B	    Light Aqua
//        C	    Light Red
//        D	    Light Purple
//        E	    Light Yellow
//        F	    Bright White

/*
Attributes	        Foreground color	   Background color

00 = normal         31 = red               40 = black
01 = bold           32 = green             41 = red
04 = underlined     33 = orange            42 = green
05 = blinking       34 = blue              43 = orange
07 = reversed       35 = purple            44 = blue
08 = concealed      36 = cyan              45 = purple
                    37 = grey              46 = cyan
                    90 = dark grey         47 = grey
                    91 = light red         100 = dark grey
                    92 = light green       101 = light red
                    93 = yellow            102 = light green
                    94 = light blue        103 = yellow
                    95 = light purple      104 = light blue
                    96 = turquoise         105 = light purple
                                           106 = turquoise
*/

#define DEBUG_LEVEL 0
#define LOG_LEVEL 3

inline std::string COLOR_STR(const std::string &str, const char *robotName)
{
    if (static_cast<std::string>(robotName).compare(0, 8, "/scout_1") == 0 || static_cast<std::string>(robotName).compare(0, 7, "scout_1") == 0)
        return "\e[0;42m" + str + "\e[m";
    else if (static_cast<std::string>(robotName).compare(0, 8, "/scout_2") == 0 || static_cast<std::string>(robotName).compare(0, 7, "scout_2") == 0)
        return "\e[0;44m" + str + "\e[m";
    else if (static_cast<std::string>(robotName).compare(0, 8, "/scout_3") == 0 || static_cast<std::string>(robotName).compare(0, 7, "scout_3") == 0)
        return "\e[0;41m" + str + "\e[m";
    return static_cast<std::string>(robotName);
}

// #define LOG_INFO2(X, node) \
//     if (LOG_LEVEL > 3) RCLCPP_INFO_STREAM(node, log_info2(X));
#define LOG_INFO(X)    \
    if (LOG_LEVEL > 2) \
        std::cout << GET_STR_STREAM("\e[0;32m" << X << "\e[m") << std::endl;
#define LOG_WARN(X)    \
    if (LOG_LEVEL > 1) \
        std::cout << GET_STR_STREAM("\e[0;93m" << X << "\e[m") << std::endl;
#define LOG_ERROR(X)   \
    if (LOG_LEVEL > 0) \
        std::cout << GET_STR_STREAM("\e[0;31m" << X << "\e[m") << std::endl;
#define LOG_FATAL(X) std::cout << GET_STR_STREAM("\e[1;37;41m" << X << "\e[m") << std::endl;

#define DEBUG(X)         \
    if (DEBUG_LEVEL > 3) \
        std::cout << GET_STR_STREAM("\e[0;33m" << X << "\e[m") << std::endl;
#define DEBUG3(X)        \
    if (DEBUG_LEVEL > 2) \
        std::cout << GET_STR_STREAM("\e[0;34m" << X << "\e[m") << std::endl;
#define DEBUG2(X)        \
    if (DEBUG_LEVEL > 1) \
        std::cout << GET_STR_STREAM("\e[0;37m" << X << "\e[m") << std::endl;
#define DEBUG1(X)        \
    if (DEBUG_LEVEL > 0) \
        std::cout << GET_STR_STREAM("\e[0;91m" << X << "\e[m") << std::endl;

#define TIME_PROFILER(functionName, seconds) \
    if (LOG_LEVEL > 0)                       \
        std::cout << GET_STR_STREAM("\033[1;94m" << functionName << " Execution Time: " << seconds << " Seconds\033[0m") << std::endl;


inline bool equateFrontiers(frontier_msgs::msg::Frontier& f1, frontier_msgs::msg::Frontier& f2, bool printValues)
{
    if (f1.initial != f2.initial || 
        f1.centroid != f2.centroid || 
        f1.middle != f2.middle || 
        f1.size != f2.size) {
        if(printValues)
        {
            std::cout << "list1.initial: " << f1.initial.x << std::endl;
            std::cout << "list1.initial: " << f1.initial.y << std::endl;
            std::cout << "list1.centroid: " << f1.centroid.x << std::endl;
            std::cout << "list1.centroid: " << f1.centroid.y << std::endl;
            std::cout << "list1.middle: " << f1.middle.x << std::endl;
            std::cout << "list1.middle: " << f1.middle.y << std::endl;
            std::cout << "list1.middle: " << f1.min_distance << std::endl;
            std::cout << "********************" << std::endl;
            std::cout << "list2.initial: " << f2.initial.x << std::endl;
            std::cout << "list2.initial: " << f2.initial.y << std::endl;
            std::cout << "list2.centroid: " << f2.centroid.x << std::endl;
            std::cout << "list2.centroid: " << f2.centroid.y << std::endl;
            std::cout << "list2.middle: " << f2.middle.x << std::endl;
            std::cout << "list2.middle: " << f2.middle.y << std::endl;
            std::cout << "list2.middle: " << f2.min_distance << std::endl;
            std::cout << "list1.size: " << f1.size << std::endl;
            std::cout << "list2.size: " << f2.size << std::endl;
            std::cout << "********************" << std::endl;
        }
        return false;
    }
    return true;
}

/** This is currently being used only to compare blacklisted frontiers. 
  * Therefore, to verify if only the traversal point is the same, 
  * we use the hash and equality as intial points.
*/
// Define hash function specialization for frontier_msgs::msg::Frontier
struct FrontierHash
{
    size_t operator()(const frontier_msgs::msg::Frontier &key) const
    {
        // Calculate hash based on some combination of member variables
        size_t hash = 0;
        // hash = std::hash<double>()(key.initial.x) ^
        //         std::hash<double>()(key.initial.y) ^
        //         std::hash<double>()(key.centroid.x) ^
        //         std::hash<double>()(key.centroid.y) ^
        //         std::hash<double>()(key.middle.x) ^
        //         std::hash<double>()(key.middle.y) ^
        //         std::hash<uint32_t>()(key.size) ^
        //         std::hash<double>()(key.min_distance) ^
        //         std::hash<double>()(key.unique_id);

        hash = std::hash<double>()(key.initial.x) ^
                std::hash<double>()(key.initial.y);
                // std::hash<double>()(key.centroid.x) ^
                // std::hash<double>()(key.centroid.y) ^
                // std::hash<double>()(key.middle.x) ^
                // std::hash<double>()(key.middle.y) ^
                // std::hash<uint32_t>()(key.size) ^
                // std::hash<double>()(key.min_distance) ^
                // std::hash<double>()(key.unique_id);
        return hash;
    }
};

// Custom equality function
struct FrontierEquality {
    bool operator()(const frontier_msgs::msg::Frontier& lhs, const frontier_msgs::msg::Frontier& rhs) const {
        // return lhs.initial == rhs.initial &&
        //         lhs.centroid == rhs.centroid &&
        //         lhs.middle == rhs.middle &&
        //         lhs.size == rhs.size &&
        //         lhs.min_distance == rhs.min_distance &&
        //         lhs.unique_id == rhs.unique_id;
        return lhs.initial == rhs.initial;
    }
};

inline double generateUID(frontier_msgs::msg::Frontier& output)
{
    return (output.initial.x * output.initial.y + output.centroid.x * output.centroid.y + output.middle.x - output.middle.y) - output.min_distance;
}
#endif // COLOR_H