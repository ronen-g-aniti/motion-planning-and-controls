#include "EnvironmentData.h"
#include <fstream>
#include <sstream>
#include <iostream>

EnvironmentData::EnvironmentData(const std::string& obstacle_geometry_file, double margin_of_safety) {

	std::ifstream file(obstacle_geometry_file);
	if (!file.is_open()) {
		throw std::runtime_error("Failed to open file");
	}

	// Parse the global reference point
	std::string dummy;
	file >> dummy >> home_latitude;
	file.ignore(std::numeric_limits<std::streamsize>::max(), ',');
	file >> dummy >> home_longitude;

	// Skip the two header lines
	std::string line;
	std::getline(file, line);
	std::getline(file, line);

	// Extract the obstacle geometry, partition by partition
	while (std::getline(file, line)) {

		std::stringstream iss(line);
		Vector3D center, halfsize;
		std::string token;

		// read center coordinates
		std::getline(iss, token, ','); center.x = std::stof(token);
		std::getline(iss, token, ','); center.y = std::stof(token);
		std::getline(iss, token, ','); center.z = std::stof(token);

		// read half-size coordinates
		std::getline(iss, token, ','); halfsize.x = std::stof(token);
		std::getline(iss, token, ','); halfsize.y = std::stof(token);
		std::getline(iss, token, ','); halfsize.z = std::stof(token);

		// add row data to class vectors
		centers.push_back(center);
		halfsizes.push_back({ halfsize.x + margin_of_safety,
							  halfsize.y + margin_of_safety,
							  halfsize.z + margin_of_safety });

		heights.push_back(center.z + halfsize.z);

		



	}
	file.close();



}

void EnvironmentData::summary() const {
	std::cout << "Environment Data Summary:" << std::endl;
	std::cout << "Home Latitude: " << home_latitude << std::endl;
	std::cout << "Home Longitude: " << home_longitude << std::endl;
	std::cout << "Margin of Safety: " << margin_of_safety << std::endl;
	std::cout << "Centers (only element 0): " << centers[0].x << ", " << centers[0].y << ", " << centers[0].z << std::endl;
	std::cout << "Half-sizes (only element 0): " << halfsizes[0].x << ", " << halfsizes[0].y << ", " << halfsizes[0].z << std::endl;

}