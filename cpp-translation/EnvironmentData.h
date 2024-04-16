#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

struct Vector3D {

	double x, y, z;
};

class EnvironmentData {

private: 
	Vector3D gps_home;
	double home_latitude;
	double home_longitude;
	double margin_of_safety;
	std::vector<Vector3D> centers;
	std::vector<Vector3D> halfsizes;
	std::vector<double> heights;
	std::vector<Vector3D> xbounds, ybounds, zbounds;
	std::vector<double> lengths; 

public:
	EnvironmentData(const std::string& obstcle_geometry_file, double margin_of_safety);
	void summary() const;
};