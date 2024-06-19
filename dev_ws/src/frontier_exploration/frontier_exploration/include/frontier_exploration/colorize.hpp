#ifndef COLOR_HPP_
#define COLOR_HPP_

#include <iostream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <frontier_exploration/Frontier.hpp>

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
    else if (static_cast<std::string>(robotName).compare(0, 8, "/scout_4") == 0 || static_cast<std::string>(robotName).compare(0, 7, "scout_3") == 0)
        return "\e[0;40m" + str + "\e[m";
    else if (static_cast<std::string>(robotName).compare(0, 8, "/scout_5") == 0 || static_cast<std::string>(robotName).compare(0, 7, "scout_3") == 0)
        return "\e[0;40m" + str + "\e[m";
    else if (static_cast<std::string>(robotName).compare(0, 8, "/scout_6") == 0 || static_cast<std::string>(robotName).compare(0, 7, "scout_3") == 0)
        return "\e[0;40m" + str + "\e[m";
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


inline bool equateFrontiers(Frontier& f1, Frontier& f2, bool printValues)
{
    if (f1.getGoalPoint() != f2.getGoalPoint() ||
        f1.getSize() != f2.getSize()) {
        if(printValues)
        {
            std::cout << "list1.goal_point: " << f1.getGoalPoint().x << std::endl;
            std::cout << "list1.goal_point: " << f1.getGoalPoint().y << std::endl;
            std::cout << "********************" << std::endl;
            std::cout << "list2.goal_point: " << f2.getGoalPoint().x << std::endl;
            std::cout << "list2.goal_point: " << f2.getGoalPoint().y << std::endl;
            std::cout << "list1.size: " << f1.getSize() << std::endl;
            std::cout << "list2.size: " << f2.getSize() << std::endl;
            std::cout << "********************" << std::endl;
        }
        return false;
    }
    return true;
}


inline bool equateFrontierList(const std::vector<Frontier> &list1, const std::vector<Frontier> &list2)
{
    bool listflag = true;
    // Check if the lists have the same size
    if (list1.size() != list2.size())
    {
        return false;
    }

    // Compare each corresponding element in both lists
    for (size_t i = 0; i < list1.size(); ++i)
    {
        auto f1 = list1[i];
        auto f2 = list2[i];
        if (!equateFrontiers(f1, f2, true))
        {
            listflag = false;
        }
    }
    return listflag;
}

/** This is currently being used only to compare blacklisted frontiers. 
  * Therefore, to verify if only the traversal point is the same, 
  * we use the hash and equality as intial points.
*/
// Define hash function specialization for Frontier
struct FrontierHash
{
    size_t operator()(const Frontier &key) const
    {
        // Calculate hash based on some combination of member variables
        size_t hash = 0;
        // hash =     std::hash<uint32_t>()(key.size) ^
        //         std::hash<double>()(key.min_distance) ^
        //         std::hash<double>()(key.unique_id);

        hash = std::hash<double>()(key.getGoalPoint().x) ^
                std::hash<double>()(key.getGoalPoint().y);
                // std::hash<uint32_t>()(key.size) ^
                // std::hash<double>()(key.min_distance) ^
                // std::hash<double>()(key.unique_id);
        return hash;
    }
};

inline size_t generateUID(std::vector<double>& output)
{
    std::hash<double> hash_fn;
    
    // Hash each double value
    std::size_t hash1 = hash_fn(output[0]);
    std::size_t hash2 = hash_fn(output[1]);

    return hash1 ^ (hash2 << 1);
    // return output[0] * output[1] + (output[1]) - output[0]
    // return output[0] + output[1];
};

template<typename T>
inline bool vectorContains(const std::vector<T>& vec, const T& value)
{
    return std::find(vec.begin(), vec.end(), value) != vec.end();
}

#endif // COLOR_H