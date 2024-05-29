#include <iostream>
#include "frontier_multirobot_allocator/hungarian/Hungarian.h"


int main(void)
{
    // please use "-std=c++11" for this initialization of vector.
	// vector< vector<double> > costMatrix = { { 0, 15, 18, 20, 25 }, 
	// 										{ 0, 20, 12, 14, 15 }, 
	// 										{ 0, 23, 25, 27, 25 }, 
	// 										{ 0, 18, 21, 23, 20 },
	// 										{ 0, 18, 16, 19, 20 } };

	vector< vector<double> > costMatrix = { { 0, 0, 0, 0, 0 } };

	// vector< vector<double> > costMatrix = { { 20, 15, 18, 20, 25 }, 
	// 										{ 18, 20, 12, 14, 15 } };

	// vector< vector<double> > costMatrix = {  { 20, 15 }, 
	//          								{ 18, 20 }, 
	//          								{ 21, 23 }, 
	//          								{ 17, 18 },
	//          								{ 18, 18 } };

	HungarianAlgorithm HungAlgo;
	vector<int> assignment;

	double cost = HungAlgo.Solve(costMatrix, assignment);

	for (unsigned int x = 0; x < costMatrix.size(); x++)
		std::cout << x << "," << assignment[x] << "\t";

	std::cout << "\ncost: " << cost << std::endl;

	return 0;
}
