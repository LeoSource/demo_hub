#pragma once

#define EIGEN_NO_DEBUG

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <thread>
#include <chrono>

using namespace std;
using namespace Eigen;

#define D2R M_PI/180.0
#define R2D 180.0/M_PI

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

	void test_lambda();

	void test_eigen_error();
	void test_nonlinear_equation();
}


