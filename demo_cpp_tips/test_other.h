#pragma once

#ifdef _WIN32
#include <Windows.h>
#include <direct.h>
#include <io.h>
#include <conio.h>
#include <thread>
#elif __linux__
#include <sys/stat.h>
#include <unistd.h>
#endif
#include <iostream>
#include <random>
#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include <functional>
#include "json.hpp"

using nlohmann::json;

namespace test_other
{
void test_other();
void test_time();
void test_enable_logic();
void test_uint_convert();
void test_directory();
void test_random_number();
void test_fill();
void convert_string();
void lambda_with_array_length();

class test_class
{
private:
	/* data */
public:
	test_class(/* args */) = default;
	~test_class() = default;
	void set_parameters(std::vector<int> v)
	{
		std::cout<<"complete setting parameters"<<std::endl;
	}
};

	
}


namespace math_tools
{
	int sign(double x);

	template<typename T>
	T limit_value(T value, T min_value, T max_value);

	template<typename T>
	T limit_min_value(T val, T min_value);

	template<typename T>
	T limit_max_value(T value, T max_value);

	template<typename UT>
	int64_t u_to_i(UT value);

	template<typename T>
	uint64_t i_to_u(T value);
}