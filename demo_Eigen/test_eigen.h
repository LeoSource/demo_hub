#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace test_eigen
{
	void array2eigenMat();
	void eigenMat2array();

	void array2eigenVec();
	void eigenVec2array();

	void vector2eigenMat();
	void eigenMat2vector();

	void vector2eigenVec();
	void eigenVec2vector();

	void test_random_matrix();

	void test_eigen();
}


