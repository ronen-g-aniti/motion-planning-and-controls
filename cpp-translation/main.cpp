#include <iostream>
#include "EnvironmentData.h"

int main() {

	const std::string obstacle_geometry_file = "colliders.csv";
	const double margin_of_safety = 5.0;
	EnvironmentData env(obstacle_geometry_file, margin_of_safety);
	env.summary();
	

	return 0;


}