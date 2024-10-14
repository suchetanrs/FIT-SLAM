
#include <iostream>
#include "frontier_exploration/FisherInfoManager.hpp"

int main(int argc, char** argv) {
    frontier_exploration::FisherInformationManager fim_manager;
    try {
        fim_manager.generateLookupTable(0.0, 20.5, -8.5  * 1.732, 8.5  * 1.732, -8.5  * 1.732, 8.5  * 1.732, 0.3);
        std::cout << "Lookup table generated successfully." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error generating lookup table: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
